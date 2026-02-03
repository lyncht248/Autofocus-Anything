#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <string>
#include <unordered_map>

class Settings {
public:
    // Constructor
    Settings(const std::string& filePath);


    // Load settings from the file
    void load();

    // Save settings to the file
    void save() const;

    // Set a setting value
    void set(const std::string& key, double value);

    // Getters for specific settings
    double getReturnPosition() const;
    double getMinPosition() const;
    double getMaxPosition() const;
    double getKp() const;
    double getKd() const;
    double getKi() const;
    double getFreq() const;
    double getFRQ2() const;
    double getPROP() const;
    double getPRO2() const;

    // Setters for specific settings
    void setReturnPosition(double value);
    void setMinPosition(double value);
    void setMaxPosition(double value);
    void setKp(double value);
    void setKd(double value);
    void setKi(double value);
    void setFreq(double value);
    void setFRQ2(double value);
    void setPROP(double value);
    void setPRO2(double value);


private:
    std::string filePath;
    std::unordered_map<std::string, double> settings;
};

#endif // SETTINGS_HPP
