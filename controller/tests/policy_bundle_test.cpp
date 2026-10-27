#include "../src/policy/PolicyBundle.hpp"
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 2) throw std::invalid_argument("usage: policy_bundle_test ROOT");
    const std::string root = argv[1];
    for (const std::string name : {"u2a_seed42", "flat_plus_gait_0917_armfree",
            "armfree/nomirror_d20", "armfree/nomirror_d25",
            "armfree/speed20_oldaction_d20", "armfree/speed20_nominal_d20"}) {
        const auto bundle = PolicyBundle::Load(root, name);
        if (bundle.name != name || bundle.rel_path != "policies/locomotion/" + name + "/policy.onnx")
            throw std::runtime_error("wrong selected bundle path: " + name);
    }
    for (const std::string name : {"", ".", "..", "../u2a_seed42", "/armfree/nomirror_d20",
            "armfree/../u2a_seed42", "armfree//nomirror_d20", "armfree/./nomirror_d20",
            "armfree/nomirror_d20/", "armfree\\nomirror_d20", "_archive/flat_plus",
            "armfree/_hidden", "armfree/name with spaces", "armfree/\nname"}) {
        if (PolicyBundle::ValidName(name)) throw std::runtime_error("accepted unsafe selector: " + name);
        bool rejected = false;
        try { PolicyBundle::Load(root, name); }
        catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("loaded unsafe selector: " + name);
    }
    std::cout << "Grouped and flat selectors load with SHA verification; invalid paths rejected\n";
}
