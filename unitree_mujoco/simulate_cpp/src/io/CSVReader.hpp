#ifndef CSV_READER_HPP
#define CSV_READER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

class CSVReader {
public:
    static std::vector<std::vector<float>> ReadCSV(const std::string& filename) {
        std::vector<std::vector<float>> data;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "[LỖI] Không thể mở file CSV: " << filename << std::endl;
            return data;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::vector<float> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                try {
                    row.push_back(std::stof(cell));
                } catch (...) {
                    // Ignore parsing errors for empty or non-numeric cells
                }
            }
            if (!row.empty()) {
                data.push_back(row);
            }
        }
        
        file.close();
        return data;
    }
};

#endif
