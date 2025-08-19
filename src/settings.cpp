#include "settings.hpp"
#include <fstream>
#include <stdexcept>
#include <algorithm> // For std::remove

const std::string DEFAULT_SETTINGS_PATH = "/home/sophia/repos/Autofocus-Anything/config/general_settings.txt";


Settings::Settings(const std::string& filePath) : filePath(filePath.empty() ? DEFAULT_SETTINGS_PATH : filePath) {}

void Settings::load() {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open settings file: " + filePath);
    }

    std::string line;
    while (std::getline(file, line)) {
        // Remove spaces and handle key = value; format
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
        size_t equalsPos = line.find('=');
        size_t semicolonPos = line.find(';');

        if (equalsPos != std::string::npos && semicolonPos != std::string::npos) {
            std::string key = line.substr(0, equalsPos);
            double value = std::stod(line.substr(equalsPos + 1, semicolonPos - equalsPos - 1));
            settings[key] = value;
        }
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

double Settings::getReturnPosition() const {
    auto it = settings.find("returnPosition");
    if (it == settings.end()) {
        throw std::runtime_error("Setting not found: returnPosition");
    }
    return it->second;
}

double Settings::getMinPosition() const {
    auto it = settings.find("MIN_POSITION");
    if (it == settings.end()) {
        throw std::runtime_error("Setting not found: MIN_POSITION");
    }
    return it->second;
}

double Settings::getMaxPosition() const {
    auto it = settings.find("MAX_POSITION");
    if (it == settings.end()) {
        throw std::runtime_error("Setting not found: MAX_POSITION");
    }
    return it->second;
}

void Settings::set(const std::string& key, double value) {
    settings[key] = value;
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
