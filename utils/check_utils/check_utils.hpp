#include <vector>
template <typename T>
bool Check_is_Init(const std::vector<T>& input){
    for (const auto& v : input) {
        if (v != 0) return true;
    }
    return false;
}

// bool Check_is_Init()