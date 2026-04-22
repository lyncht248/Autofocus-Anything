#include "settings.hpp"
#include <algorithm> // For std::remove
#include <fstream>
#include <stdexcept>

#include <iostream>

const std::string DEFAULT_SETTINGS_PATH = "../config/general_settings.txt";

Settings::Settings(const std::string &filePath)
    : filePath(filePath.empty() ? DEFAULT_SETTINGS_PATH : filePath) {}

void Settings::load() {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open settings file: " + filePath);
  }

  std::cout << "Loading settings from " << filePath << std::endl;

  std::string line;
  while (std::getline(file, line)) {
    // Handle key=value format (no spaces around equals sign)
    size_t equalsPos = line.find('=');

    if (equalsPos != std::string::npos) {
      std::string key = line.substr(0, equalsPos);
      std::string valueStr = line.substr(equalsPos + 1);

      // Trim whitespace from key and value
      key.erase(0, key.find_first_not_of(" \t"));
      key.erase(key.find_last_not_of(" \t") + 1);
      valueStr.erase(0, valueStr.find_first_not_of(" \t"));
      valueStr.erase(valueStr.find_last_not_of(" \t") + 1);

      // Remove any trailing semicolon if present
      size_t semicolonPos = valueStr.find(';');
      if (semicolonPos != std::string::npos) {
        valueStr = valueStr.substr(0, semicolonPos);
      }

      double value = std::stod(valueStr);
      settings[key] = value;
    }
  }

  file.close();
}

void Settings::save() const {
  std::ofstream file(filePath);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open settings file for writing: " +
                             filePath);
  }

  for (const auto &[key, value] : settings) {
    file << key << "=" << value << "\n";
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

double Settings::getKp() const {
  auto it = settings.find("Kp");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: Kp");
  }
  return it->second;
}

double Settings::getKd() const {
  auto it = settings.find("Kd");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: Kd");
  }
  return it->second;
}

double Settings::getKi() const {
  auto it = settings.find("Ki");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: Ki");
  }
  return it->second;
}

double Settings::getFreq() const {
  auto it = settings.find("FREQ");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: FREQ");
  }
  return it->second;
}

double Settings::getFRQ2() const {
  auto it = settings.find("FRQ2");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: FRQ2");
  }
  return it->second;
}

double Settings::getPROP() const {
  auto it = settings.find("PROP");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: PROP");
  }
  return it->second;
}

double Settings::getPRO2() const {
  auto it = settings.find("PRO2");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: PRO2");
  }
  return it->second;
}

double Settings::getSSPD() const {
  auto it = settings.find("SSPD");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: SSPD");
  }
  return it->second;
}

double Settings::getACCE() const {
  auto it = settings.find("ACCE");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: ACCE");
  }
  return it->second;
}

double Settings::getDECE() const {
  auto it = settings.find("DECE");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: DECE");
  }
  return it->second;
}

double Settings::getMASS() const {
  auto it = settings.find("MASS");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: MASS");
  }
  return it->second;
}

double Settings::getPTOL() const {
  auto it = settings.find("PTOL");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: PTOL");
  }
  return it->second;
}

double Settings::getPTO2() const {
  auto it = settings.find("PTO2");
  if (it == settings.end()) {
    throw std::runtime_error("Setting not found: PTO2");
  }
  return it->second;
}

void Settings::set(const std::string &key, double value) {
  settings[key] = value;
}

void Settings::setReturnPosition(double value) { set("returnPosition", value); }

void Settings::setMinPosition(double value) { set("MIN_POSITION", value); }

void Settings::setMaxPosition(double value) { set("MAX_POSITION", value); }

void Settings::setKp(double value) { set("Kp", value); }
void Settings::setKd(double value) { set("Kd", value); }
void Settings::setKi(double value) { set("Ki", value); }

void Settings::setFreq(double value) { set("FREQ", value); }

void Settings::setFRQ2(double value) { set("FRQ2", value); }

void Settings::setPROP(double value) { set("PROP", value); }
void Settings::setPRO2(double value) { set("PRO2", value); }
void Settings::setSSPD(double value) { set("SSPD", value); }
void Settings::setACCE(double value) { set("ACCE", value); }
void Settings::setDECE(double value) { set("DECE", value); }
void Settings::setMASS(double value) { set("MASS", value); }
void Settings::setPTOL(double value) { set("PTOL", value); }
void Settings::setPTO2(double value) { set("PTO2", value); }
