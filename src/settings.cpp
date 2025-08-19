#include "settings.hpp"
#include <fstream>
#include <stdexcept>

Settings::Settings(const std::string& filePath) : filePath(filePath) {}

void Settings::load() {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open settings file: " + filePath);
    }

    std::string key;
    double value;
    while (file >> key >> value) {
        settings[key] = value;
    }

    file.close();
}

void Settings::save() const {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open settings file for writing: " + filePath);
    }

    for (const auto& [key, value] : settings) {
        file << key << " " << value << "\n";
    }

    file.close();
}

double Settings::get(const std::string& key) const {
    auto it = settings.find(key);
    if (it == settings.end()) {
        throw std::runtime_error("Setting not found: " + key);
    }
    return it->second;
}

void Settings::set(const std::string& key, double value) {
    settings[key] = value;
}

double Settings::getReturnPosition() const {
    return get("returnPosition");
}

double Settings::getMinPosition() const {
    return get("MIN_POSITION");
}

double Settings::getMaxPosition() const {
    return get("MAX_POSITION");
}

void Settings::setReturnPosition(double value) {
    set("returnPosition", value);
}

void Settings::setMinPosition(double value) {
    set("MIN_POSITION", value);
}

void Settings::setMaxPosition(double value) {
    set("MAX_POSITION", value);
}
