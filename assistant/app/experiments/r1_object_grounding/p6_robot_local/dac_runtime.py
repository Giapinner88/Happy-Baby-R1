"""Robot-only TensorRT DAC inference, NumPy geometry, no Torch or network."""
import ctypes
import json
from pathlib import Path
import time
import numpy as np
import cv2


class Geometry:
    def __init__(self,path,backend='opencv'):
        if backend not in ('opencv','numpy_exact'):raise ValueError('invalid remap backend')
        self.backend=backend
        z=np.load(path,allow_pickle=False)
        self.size=tuple(int(x) for x in z['source_size']);self.forward=tuple(int(x) for x in z['forward_size'])
        self.rays=z['rays'];self.scale=float(z['scale']);self.active=z['erp_active']
        self.lat=z['lat_range'];self.lon=z['lon_range'];grid=z['grid']
        # align_corners=True and border padding, matching upstream grid_sample.
        x=np.clip((grid[...,0]+1)*.5*(self.size[0]-1),0,self.size[0]-1)
        y=np.clip((grid[...,1]+1)*.5*(self.size[1]-1),0,self.size[1]-1)
        self.sample_x=x.astype(np.float32);self.sample_y=y.astype(np.float32)
        self.x0=np.floor(x).astype(np.int32);self.y0=np.floor(y).astype(np.int32)
        self.x1=np.minimum(self.x0+1,self.size[0]-1);self.y1=np.minimum(self.y0+1,self.size[1]-1)
        self.wx=(x-self.x0).astype(np.float32)[...,None];self.wy=(y-self.y0).astype(np.float32)[...,None]
        self.grid_active=((grid[...,0]>-1)&(grid[...,0]<1)&(grid[...,1]>-1)&(grid[...,1]<1)).astype(np.float32)[...,None]
        h,w=self.forward
        longitude=np.arctan2(self.rays[...,0],self.rays[...,2]);latitude=np.arctan2(self.rays[...,1],np.hypot(self.rays[...,0],self.rays[...,2]))
        self.mx=((longitude-self.lon[0])/(self.lon[1]-self.lon[0])*(w-1)).astype(np.float32)
        self.my=((latitude-self.lat[0])/(self.lat[1]-self.lat[0])*(h-1)).astype(np.float32)
        self.valid=(self.mx>=0)&(self.mx<=w-1)&(self.my>=0)&(self.my<=h-1)&(self.rays[...,2]>0)
        support=cv2.remap(self.active,self.mx,self.my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        self.valid &= support>.999

    def prepare(self,image):
        if (image.shape[1],image.shape[0])!=self.size:raise ValueError('camera dimensions changed')
        rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB).astype(np.float32)/255.
        if self.backend=='opencv':
            sampled=cv2.remap(rgb,self.sample_x,self.sample_y,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
        else:
            a=rgb[self.y0,self.x0]*(1-self.wx)+rgb[self.y0,self.x1]*self.wx
            b=rgb[self.y1,self.x0]*(1-self.wx)+rgb[self.y1,self.x1]*self.wx
            sampled=a*(1-self.wy)+b*self.wy
        erp=(sampled*self.grid_active*255).astype(np.uint8)
        erp=cv2.resize(erp,(self.forward[1],self.forward[0]),interpolation=cv2.INTER_LINEAR)
        normalized=(erp.astype(np.float32)/255.-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
        return np.ascontiguousarray(normalized.transpose(2,0,1)[None]),erp

    def restore(self,prediction):
        erp=np.asarray(prediction,dtype=np.float32).reshape(self.forward)*self.scale
        if not np.isfinite(erp).all() or (erp<=0).any():raise ValueError('invalid depth prediction')
        depth=cv2.remap(erp,self.mx,self.my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        valid=self.valid & np.isfinite(depth)&(depth>0);depth[~valid]=np.nan
        return depth,valid


class TensorRT:
    def __init__(self,engine_path):
        import tensorrt as trt
        self.cuda=ctypes.CDLL('/usr/local/cuda/lib64/libcudart.so')
        self.cuda.cudaMalloc.argtypes=[ctypes.POINTER(ctypes.c_void_p),ctypes.c_size_t]
        self.cuda.cudaFree.argtypes=[ctypes.c_void_p]
        self.cuda.cudaMemcpy.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t,ctypes.c_int]
        self.cuda.cudaDeviceSynchronize.argtypes=[]
        self.check(self.cuda.cudaFree(ctypes.c_void_p()))
        self.runtime=trt.Runtime(trt.Logger(trt.Logger.WARNING))
        self.engine=self.runtime.deserialize_cuda_engine(Path(engine_path).read_bytes())
        if self.engine is None:raise RuntimeError('TensorRT engine deserialize failed')
        self.context=self.engine.create_execution_context();self.pointers=[];self.host=[]
        if self.engine.num_bindings!=2:raise ValueError('unexpected DAC bindings')
        for i in range(self.engine.num_bindings):
            shape=tuple(self.engine.get_binding_shape(i))
            if min(shape)<=0:raise ValueError('static bindings required')
            dtype=trt.nptype(self.engine.get_binding_dtype(i));array=np.empty(shape,dtype=dtype)
            p=ctypes.c_void_p();self.check(self.cuda.cudaMalloc(ctypes.byref(p),array.nbytes))
            self.pointers.append(p);self.host.append(array)
        self.input=next(i for i in range(2) if self.engine.binding_is_input(i));self.output=1-self.input

    def check(self,status):
        if status:raise RuntimeError('CUDA error '+str(status))

    def infer(self,image):
        a=self.host[self.input]
        if image.shape!=a.shape:raise ValueError('DAC input shape')
        np.copyto(a,image,casting='same_kind')
        self.check(self.cuda.cudaMemcpy(self.pointers[self.input],ctypes.c_void_p(a.ctypes.data),a.nbytes,1))
        if not self.context.execute_v2([p.value for p in self.pointers]):raise RuntimeError('TensorRT execute failed')
        b=self.host[self.output]
        self.check(self.cuda.cudaMemcpy(ctypes.c_void_p(b.ctypes.data),self.pointers[self.output],b.nbytes,2))
        return b.copy()

    def close(self):
        for pointer in self.pointers:self.check(self.cuda.cudaFree(pointer))
        self.pointers=[]
