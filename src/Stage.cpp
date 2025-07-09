#include "Stage.h"

const LinearStage XLS_312{"XLS1=312", 312.5, 1000};
const LinearStage XLS_1250{"XLS1=1250", 312.5, 1000};
const LinearStage XLS_78{"XLS1=78", 78.125, 1000};
const LinearStage XLS_5{"XLS1=5", 5, 1000};
const LinearStage XLS_1{"XLS1=1", 1, 1000};
const LinearStage XLS_312_3N{"XLS3=312", 312.5, 1000};
const LinearStage XLS_1250_3N{"XLS3=1250", 312.5, 1000};
const LinearStage XLS_78_3N{"XLS3=78", 78.125, 1000};
const LinearStage XLS_5_3N{"XLS3=5", 5, 1000};
const LinearStage XLS_1_3N{"XLS3=1", 1, 1000};
const LinearStage XLA_312{"XLA1=312", 312.5, 1000};
const LinearStage XLA_1250{"XLA1=1250", 1250, 1000};
const LinearStage XLA_78{"XLA1=78", 78.125, 1000};
const LinearStage XLA_312_3N{"XLA3=312", 312.5, 1000};
const LinearStage XLA_1250_3N{"XLA3=1250", 1250, 1000};
const LinearStage XLA_78_3N{"XLA3=78", 78.125, 1000};

const RotationStage XRTA{"XRTA=109", 2 * M_PI * 1000000 / 57600, 100, 57600};
const RotationStage XRTU_40_3{"XRT1=2", 2 * M_PI * 1000000 / 86400, 100, 86400};
const RotationStage XRTU_40_19{"XRT1=18", 2 * M_PI * 1000000 / 86400, 100, 86400};
const RotationStage XRTU_40_49{"XRT1=47", 2 * M_PI * 1000000 / 86400, 100, 86400};
const RotationStage XRTU_40_73{"XRT1=73", 2 * M_PI * 1000000 / 86400, 100, 86400};
const RotationStage XRTU_30_3{"XRT1=3", 2 * M_PI * 1000000 / 1843200, 100, 1843200};
const RotationStage XRTU_30_19{"XRT1=19", 2 * M_PI * 1000000 / 360000, 100, 360000};
const RotationStage XRTU_30_49{"XRT1=49", 2 * M_PI * 1000000 / 144000, 100, 144000};
const RotationStage XRTU_30_109{"XRT1=109", 2 * M_PI * 1000000 / 57600, 100, 57600};
