#ifndef _AXIS_H
#define _AXIS_H

#include "Xeryon.h"
#include "Stage.h"

#include <string>
#include <ctime>
#include <map>
#include <unordered_set>

const int DEFAULT_POLI_VALUE = 200;

class Xeryon;
class Stage;

class Axis 
{
    public:
        Axis(Xeryon * xeryon, const char letter, const Stage * stage) :
            xeryon_(xeryon), letter_(letter), stage_(stage),
            def_poli_value_(DEFAULT_POLI_VALUE)
        {
            data_["DPOS"] = 0;
            data_["EPOS"] = 0;
            data_["STAT"] = 0;
            not_settings_command_= { "DPOS", "EPOS", "HOME", "ZERO", "RSET", "INDX", "STEP", "MOVE", "STOP", "CONT" };
            was_valid_DPOS_ = false; // Initialize at false because when the stage starts, DPOS was not valid.
            is_logging_ = false;
        }

        void findIndex();
        void setDPOS(Distance d);
        Distance getDPOS() { return Distance(data_["DPOS"] * stage_->getEncoderResolution(), Distance::NM); }
        Distance getEPOS() { return Distance(data_["EPOS"] * stage_->getEncoderResolution(), Distance::NM); }
        void step(Distance d);

        void startLogging();
        void endLogging();

        int getFrequency() { return data_["FREQ"]; }

        void startScan(int direction, int timeout = 0);
        void stopScan();

        void calibrate();
        void setSpeed(Distance speed); // speed in Distance per second

        void setPTOL(int value) { setSetting("PTOL", value); }
        void setPTO2(int value) { setSetting("PTO2", value); }

        bool isForceZero() { return (data_["STAT"] & (1 << 4)); }
        bool isMotorOn() { return (data_["STAT"] & (1 << 5)); }
        bool isClosedLoop() { return (data_["STAT"] & (1 << 6)); }
        bool isEncoderAtIndex() { return (data_["STAT"] & (1 << 7)); }
        bool isEncoderValid() { return (data_["STAT"] & (1 << 8)); }
        bool isSearchingIndex() { return (data_["STAT"] & (1 << 9)); }
        bool isPositionReached() { return (data_["STAT"] & (1 << 10)); }
        bool isEncoderError() { return (data_["STAT"] & (1 << 12)); }
        bool isScanning() { return (data_["STAT"] & (1 << 13)); }
        bool isAtLeftEnd() { return (data_["STAT"] & (1 << 14)); }
        bool isAtRightEnd() { return (data_["STAT"] & (1 << 15)); }
        bool isErrorLimit() { return (data_["STAT"] & (1 << 16)); }
        bool isSearchingOptimalFrequency() { return (data_["STAT"] & (1 << 17)); }

        char getLetter() { return letter_; }

        void sendCommand(std::string tag, int value);

        void sendSettings();
        void setSetting(std::string tag, std::string value, bool fromSettingsFile = false);
        void setSetting(std::string tag, int value, bool fromSettingsFile = false);
        int getSetting(std::string tag) { return settings_[tag]; }
        int getData(std::string tag) { return data_[tag]; }
        void stopMovement();
        void stop();
        void reset();
        void receiveData(std::string tag, int value);
    private:
        const char letter_;
        Xeryon * xeryon_;
        const Stage * stage_;
        std::map<std::string, int> settings_;
        std::map<std::string, int> data_;
        int def_poli_value_;
        int update_nb_;
        bool was_valid_DPOS_;
        std::unordered_set<std::string> not_settings_command_;
        bool is_logging_;

        int applySettingsMultipliers_(std::string tag, int value);
        bool isWithinTol_(int DPOS);
        std::time_t getActualTime_();
        bool timeOutReached_(std::time_t start_time, int distance);
        void waitForUpdate_();
        void sendCommand_(const char * command);
        void sendCommand_(std::string tag, int value);
        void handleEnableSequence_();
};
#endif
