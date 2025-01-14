#ifndef _STAGE_H
#define _STAGE_H

#define _USE_MATH_DEFINES
#include <cmath>

const double AMPLITUDE_MULTIPLIER = 1456.0;
const double PHASE_MULTIPLIER = 182;

class Stage
{
    public:
        Stage(const char * encoderResolutionCommand, double encoderResolution, double speedMultiplier) :
            encoderResolutionCommand_(encoderResolutionCommand), encoderResolution_(encoderResolution),
            speedMultiplier_(speedMultiplier), amplitudeMultiplier_(AMPLITUDE_MULTIPLIER),
            phaseMultiplier_(PHASE_MULTIPLIER) { }
        virtual ~Stage() { }

        const char * getEncoderResolutionCommand() const { return encoderResolutionCommand_; }
        double getEncoderResolution() const { return encoderResolution_; }
        double getSpeedMultiplier() const { return speedMultiplier_; }
        double getAmplitudeMultiplier() const { return amplitudeMultiplier_; }
        double getPhaseMultiplier() const { return phaseMultiplier_; }
        virtual bool isLinear() const { return false; }

    private:
        const char * encoderResolutionCommand_;
        double encoderResolution_; 
        double speedMultiplier_;
        double amplitudeMultiplier_;
        double phaseMultiplier_;
};

class LinearStage : public Stage
{
    public:
        LinearStage(const char * encoderResolutionCommand, double encoderResolution, double speedMultiplier) :
            Stage(encoderResolutionCommand, encoderResolution, speedMultiplier) { }
        bool isLinear() const { return true; }
};

class RotationStage : public Stage
{
    public:
        RotationStage(const char * encoderResolutionCommand, double encoderResolution, double speedMultiplier, double encCountsPerRev) :
            Stage(encoderResolutionCommand, encoderResolution, speedMultiplier), encCountsPerRev_(encCountsPerRev) { }
        bool isLinear() const { return false; }

    private:
        double encCountsPerRev_;
};

extern const LinearStage XLS_312;
extern const LinearStage XLS_1250;
extern const LinearStage XLS_78;
extern const LinearStage XLS_5;
extern const LinearStage XLS_1;
extern const LinearStage XLS_312_3N;
extern const LinearStage XLS_1250_3N;
extern const LinearStage XLS_78_3N;
extern const LinearStage XLS_5_3N;
extern const LinearStage XLS_1_3N;
extern const LinearStage XLA_312;
extern const LinearStage XLA_1250;
extern const LinearStage XLA_78;
extern const RotationStage XRTA;
extern const RotationStage XRTU_30_109;
extern const RotationStage XRTU_40_73;
extern const RotationStage XRTU_40_3;
extern const LinearStage XLA_1250_3N;
#endif

// vim: expandtab:ts=4:sw=4
