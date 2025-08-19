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

    // Get a setting value
    double get(const std::string& key) const;

    // Set a setting value
    void set(const std::string& key, double value);

    // Getters for specific settings
    double getReturnPosition() const;
    double getMinPosition() const;
    double getMaxPosition() const;

    // Setters for specific settings
    void setReturnPosition(double value);
    void setMinPosition(double value);
    void setMaxPosition(double value);

private:
    std::string filePath;
    std::unordered_map<std::string, double> settings;
};

#endif // SETTINGS_HPP
