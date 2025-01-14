#include "Distance.h"

Distance operator"" _mm(long double val) { return Distance(val * 1000000.0); }
Distance operator"" _mu(long double val) { return Distance(val * 1000.0); }
Distance operator"" _nm(long double val) { return Distance(val); }
Distance operator"" _inch(long double val) { return Distance(val * 25.4 * 1000000.0); }
Distance operator"" _minch(long double val) { return Distance(val * 25.4 * 1000.0); }

Distance operator"" _mm(unsigned long long val) { return Distance(val * 1000000.0); }
Distance operator"" _mu(unsigned long long val) { return Distance(val * 1000.0); }
Distance operator"" _nm(unsigned long long val) { return Distance(val); }
Distance operator"" _inch(unsigned long long val) { return Distance(val * 25.4 * 1000000.0); }
Distance operator"" _minch(unsigned long long val) { return Distance(val * 25.4 * 1000.0); }

#if 0

    def convertUnitsToEncoder(self, value, units):
        elif units == Units.mrad:
            return round(value * 10 ** 3 * 1 / self.stage.encoderResolution)
        elif units == Units.rad:
            return round(value * 10 ** 6 * 1 / self.stage.encoderResolution)
        elif units == Units.deg:
            return round(value * (2 * math.pi) / 360 * 10 ** 6 / self.stage.encoderResolution)
        else:
            self.xeryon_object.stop()
            raise ("Unexpected unit")

    def convertEncoderUnitsToUnits(self, value, units):
        elif units == Units.mrad:
            return value / (10 ** 3 * 1 / self.stage.encoderResolution)
        elif units == Units.rad:
            return value / (10 ** 6 * 1 / self.stage.encoderResolution)
        elif units == Units.deg:
            return value / ((2 * math.pi) / 360 * 10 ** 6 / self.stage.encoderResolution)
        else:
            self.xeryon_object.stop()
            raise ("Unexpected unit")
#endif

