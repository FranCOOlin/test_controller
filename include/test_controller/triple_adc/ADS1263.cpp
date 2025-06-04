#include "ADS1263.h"
#include <stdio.h>

// 全局变量，区分扫描模式：0 单端输入，1 差分输入
static UBYTE ScanMode = 0;


/***************************************
* 函数: writeCmd
* 参数: Cmd - 命令字
* 功能: 发送命令
***************************************/
void ADS1263::writeCmd(UBYTE Cmd)
{
    DEV_GPIO_Write(DEV_CS_PIN, DEV_GPIO_LOW);
    DEV_HARDWARE_SPI_TransferByte(Cmd);
    DEV_GPIO_Write(DEV_CS_PIN, DEV_GPIO_HIGH);
}

/***************************************
* 函数: writeReg
* 参数: Reg - 目标寄存器, data - 写入数据
* 功能: 写数据到寄存器
***************************************/
void ADS1263::writeReg(UBYTE Reg, UBYTE data)
{
    DEV_GPIO_Write(DEV_CS_PIN, DEV_GPIO_LOW);
    DEV_HARDWARE_SPI_TransferByte(CMD_WREG | Reg);
    DEV_HARDWARE_SPI_TransferByte(0x00);
    DEV_HARDWARE_SPI_TransferByte(data);
    DEV_GPIO_Write(DEV_CS_PIN, DEV_GPIO_HIGH);
}

/***************************************
* 函数: readData
* 参数: Reg - 目标寄存器
* 功能: 读取寄存器数据，返回 1 字节数据
***************************************/
UBYTE ADS1263::readData(UBYTE Reg)
{
    UBYTE temp = 0;
    DEV_GPIO_Write(DEV_CS_PIN, DEV_GPIO_LOW);
    DEV_HARDWARE_SPI_TransferByte(CMD_RREG | Reg);
    DEV_HARDWARE_SPI_TransferByte(0x00);
    temp = DEV_HARDWARE_SPI_ReadByte();
    DEV_GPIO_Write(DEV_CS_PIN, DEV_GPIO_HIGH);
    return temp;
}

/***************************************
* 函数: checkSum
* 参数: val - 4 字节数据, byt - 校验字节
* 功能: 计算校验，返回 0 表示校验成功
***************************************/
UBYTE ADS1263::checkSum(UDOUBLE val, UBYTE byt)
{
    UBYTE sum = 0;
    UBYTE mask = 0xFF;  // 8 位掩码
    while(val) {
        sum += val & mask;
        val >>= 8;
    }
    sum += 0x9b;
    return sum ^ byt;
}

/***************************************
* 函数: waitDRDY
* 功能: 等待 DRDY（数据就绪）结束，超时则打印信息
***************************************/
void ADS1263::waitDRDY(void)
{
    UDOUBLE i = 0;
    while(1) {
        if(DEV_GPIO_Read(DEV_DRDY_PIN) == DEV_GPIO_LOW)
            break;
        if(i >= 4000000) {
            ROS_INFO("Time Out ...");
            break;
        }
        i++;
    }
}

/***************************************
* 函数: ReadChipID
* 功能: 读取设备ID
***************************************/
UBYTE ADS1263::readChipID(void)
{
    UBYTE id;
    id = readData(REG_ID);
    return id >> 5;
}

/***************************************
* 函数: setMode
* 参数: Mode - 0 单端输入, 非0 差分输入
* 功能: 设置扫描模式
***************************************/
void ADS1263::setMode(UBYTE Mode)
{
    if(Mode == 0) {
        ScanMode = 0;
    } else {
        ScanMode = 1;
    }
}

/***************************************
* 函数: configADC1
* 参数: gain, drate, delay
* 功能: 配置 ADC1 的增益、采样速率及延时
***************************************/
void ADS1263::configADC1(ADS1263_GAIN gain, ADS1263_DRATE drate, ADS1263_DELAY delay)
{
    UBYTE MODE2 = 0x80;             // 0x80: PGA bypassed
    MODE2 |= (gain << 4) | drate;
    writeReg(REG_MODE2, MODE2);
    DEV_Delay_ms(1);
    if(readData(REG_MODE2) == MODE2)
        ROS_INFO("REG_MODE2 success ");
    else
        ROS_INFO("REG_MODE2 unsuccess ");
    
    UBYTE REFMUX = 0x24;        // 0x24: VDD,VSS as REF
    writeReg(REG_REFMUX, REFMUX);
    DEV_Delay_ms(1);
    if(readData(REG_REFMUX) == REFMUX)
        ROS_INFO("REG_REFMUX success ");
    else
        ROS_INFO("REG_REFMUX unsuccess ");
    
    UBYTE MODE0 = delay;
    writeReg(REG_MODE0, MODE0); 
    DEV_Delay_ms(1);
    if(readData(REG_MODE0) == MODE0)
        ROS_INFO("REG_MODE0 success ");
    else
        ROS_INFO("REG_MODE0 unsuccess ");
    
    UBYTE MODE1 = 0x84; // 0x84: FIR digital filter
    writeReg(REG_MODE1, MODE1); 
    DEV_Delay_ms(1);
    if(readData(REG_MODE1) == MODE1)
        ROS_INFO("REG_MODE1 success ");
    else
        ROS_INFO("REG_MODE1 unsuccess ");
}

/***************************************
* 函数: configADC2
* 参数: gain, drate, delay
* 功能: 配置 ADC2 的增益、采样速率及延时
***************************************/
void ADS1263::configADC2(ADS1263_ADC2_GAIN gain, ADS1263_ADC2_DRATE drate, ADS1263_DELAY delay)
{
    UBYTE ADC2CFG = 0x20;               // 0x20: VAVDD and VAVSS as REF
    ADC2CFG |= (drate << 6) | gain;
    writeReg(REG_ADC2CFG, ADC2CFG);
    DEV_Delay_ms(1);
    if(readData(REG_ADC2CFG) == ADC2CFG)
        ROS_INFO("REG_ADC2CFG success ");
    else
        ROS_INFO("REG_ADC2CFG unsuccess ");
    
    UBYTE MODE0 = delay;
    writeReg(REG_MODE0, MODE0); 
    DEV_Delay_ms(1);
    if(readData(REG_MODE0) == MODE0)
        ROS_INFO("REG_MODE0 success ");
    else
        ROS_INFO("REG_MODE0 unsuccess ");
}

/***************************************
* 函数: initADC1
* 参数: rate - ADC1 采样速率
* 功能: 初始化 ADC1
***************************************/
UBYTE ADS1263::initADC1(ADS1263_DRATE rate)
{
    softReset();
    if(readChipID() == 0) {
        ROS_INFO("ID Read success ");
    } else {
        ROS_INFO("ID Read failed ");
        return 1;
    }
    softStop();
    configADC1(ADS1263_GAIN_1, rate, ADS1263_DELAY_35us);
    softStart();
    return 0;
}

/***************************************
* 函数: initADC2
* 参数: rate - ADC2 采样速率
* 功能: 初始化 ADC2
***************************************/
UBYTE ADS1263::initADC2(ADS1263_ADC2_DRATE rate)
{
    softReset();
    if(readChipID() == 1) {
        ROS_INFO("ID Read success ");
    } else {
        ROS_INFO("ID Read failed ");
        return 1;
    }
    writeCmd(CMD_STOP2);
    configADC2(ADS1263_ADC2_GAIN_1, rate, ADS1263_DELAY_35us);
    return 0;
}

/***************************************
* 函数: setChannal
* 参数: Channal - 通道号
* 功能: 设置 ADC1 通道
***************************************/
void ADS1263::setChannal(UBYTE Channal)
{
    if(Channal > 10)
        return;
    UBYTE INPMUX = (Channal << 4) | 0x0a; // 0x0a: VCOM 作为负输入
    writeReg(REG_INPMUX, INPMUX);
    if(readData(REG_INPMUX) != INPMUX)
        ROS_INFO("setChannal unsuccess ");
}

/***************************************
* 函数: setChannal_ADC2
* 参数: Channal - 通道号
* 功能: 设置 ADC2 通道
***************************************/
void ADS1263::setChannal_ADC2(UBYTE Channal)
{
    if(Channal > 10)
        return;
    UBYTE INPMUX = (Channal << 4) | 0x0a;
    writeReg(REG_ADC2MUX, INPMUX);
    if(readData(REG_ADC2MUX) != INPMUX)
        ROS_INFO("setChannal_ADC2 unsuccess ");
}

/***************************************
* 函数: setDiffChannal
* 参数: Channal - 差分通道号（0~4）
* 功能: 设置 ADC1 差分通道
***************************************/
void ADS1263::setDiffChannal(UBYTE Channal)
{
    UBYTE INPMUX;
    if(Channal == 0)
        INPMUX = (0 << 4) | 1;    // AIN0-AIN1
    else if(Channal == 1)
        INPMUX = (2 << 4) | 3;    // AIN2-AIN3
    else if(Channal == 2)
        INPMUX = (4 << 4) | 5;    // AIN4-AIN5
    else if(Channal == 3)
        INPMUX = (6 << 4) | 7;    // AIN6-AIN7
    else if(Channal == 4)
        INPMUX = (8 << 4) | 9;    // AIN8-AIN9
    writeReg(REG_INPMUX, INPMUX);
    if(readData(REG_INPMUX) != INPMUX)
        ROS_INFO("setDiffChannal unsuccess ");
}

/***************************************
* 函数: setDiffChannal_ADC2
* 参数: Channal - 差分通道号（0~4）
* 功能: 设置 ADC2 差分通道
***************************************/
void ADS1263::setDiffChannal_ADC2(UBYTE Channal)
{
    UBYTE INPMUX;
    if(Channal == 0)
        INPMUX = (0 << 4) | 1;
    else if(Channal == 1)
        INPMUX = (2 << 4) | 3;
    else if(Channal == 2)
        INPMUX = (4 << 4) | 5;
    else if(Channal == 3)
        INPMUX = (6 << 4) | 7;
    else if(Channal == 4)
        INPMUX = (8 << 4) | 9;
    writeReg(REG_ADC2MUX, INPMUX);
    if(readData(REG_ADC2MUX) != INPMUX)
        ROS_INFO("setDiffChannal_ADC2 unsuccess ");
}

/***************************************
* 函数: readADC1Data
* 功能: 读取 ADC1 数据（4 字节数据）
***************************************/
double ADS1263::readADC1Data(void)
{
    int read = 0;
    UBYTE buf[4] = {0};
    UBYTE Status, CRC;
    //ROS_INFO("1");
    DEV_GPIO_Write(DEV_CS_PIN, 0);
    
    //ROS_INFO("2");
    do {
        DEV_HARDWARE_SPI_TransferByte(CMD_RDATA1);
        Status = DEV_HARDWARE_SPI_ReadByte();
        //ROS_INFO("3");
    } while((Status & 0x40) == 0);
    buf[0] = DEV_HARDWARE_SPI_ReadByte();
    //ROS_INFO("4");
    buf[1] = DEV_HARDWARE_SPI_ReadByte();
    //ROS_INFO("5");
    buf[2] = DEV_HARDWARE_SPI_ReadByte();
    //ROS_INFO("6");
    buf[3] = DEV_HARDWARE_SPI_ReadByte();
    //ROS_INFO("7");
    CRC = DEV_HARDWARE_SPI_ReadByte();
    DEV_GPIO_Write(DEV_CS_PIN, 1);
    read |= ((UDOUBLE)buf[0] << 24);
    read |= ((UDOUBLE)buf[1] << 16);
    read |= ((UDOUBLE)buf[2] << 8);
    read |= (UDOUBLE)buf[3];
    if(checkSum(read, CRC) != 0)
        ROS_INFO("ADC1 Data read error! ");
    double v;
    if(read>>31 == 1){
        v = REF*2 - read * REF / 0x80000000;
    }
    else{
        v = -read * REF / 0x7fffffff;
    }
    return v;
}

/***************************************
* 函数: readADC2Data
* 功能: 读取 ADC2 数据（3 字节数据）
***************************************/
UDOUBLE ADS1263::readADC2Data(void)
{
    UDOUBLE read = 0;
    UBYTE buf[4] = {0};
    UBYTE Status, CRC;
    DEV_GPIO_Write(DEV_CS_PIN, 0);
    do {
        DEV_HARDWARE_SPI_TransferByte(CMD_RDATA2);
        Status = DEV_HARDWARE_SPI_ReadByte();
    } while((Status & 0x80) == 0);
    
    buf[0] = DEV_HARDWARE_SPI_ReadByte();
    buf[1] = DEV_HARDWARE_SPI_ReadByte();
    buf[2] = DEV_HARDWARE_SPI_ReadByte();
    buf[3] = DEV_HARDWARE_SPI_ReadByte();
    CRC = DEV_HARDWARE_SPI_ReadByte();
    DEV_GPIO_Write(DEV_CS_PIN, 1);
    read |= ((UDOUBLE)buf[0] << 16);
    read |= ((UDOUBLE)buf[1] << 8);
    read |= (UDOUBLE)buf[2];
    if(checkSum(read, CRC) != 0)
        ROS_INFO("ADC2 Data read error! ");
    return read;
}

/***************************************
* 函数: getChannalValue
* 参数: Channel - 通道号
* 功能: 读取 ADC1 指定通道数据，依据 ScanMode 选择单端或差分通道
***************************************/
UDOUBLE ADS1263::getChannalValue(UBYTE Channel)
{
    UDOUBLE Value = 0;
    if(ScanMode == 0) { // 单端输入
        if(Channel > 10)
            return 0;
        setChannal(Channel);
        waitDRDY();
        Value = readADC1Data();
    } else { // 差分输入
        if(Channel > 4)
            return 0;
        setDiffChannal(Channel);
        waitDRDY();
        Value = readADC1Data();
    }
    return Value;
}

/***************************************
* 函数: getChannalValueADC2
* 参数: Channel - 通道号
* 功能: 读取 ADC2 指定通道数据
***************************************/
UDOUBLE ADS1263::getChannalValueADC2(UBYTE Channel)
{
    UDOUBLE Value = 0;
    if(ScanMode == 0) {
        if(Channel > 10)
            return 0;
        setChannal_ADC2(Channel);
        writeCmd(CMD_START2);
        Value = readADC2Data();
    } else {
        if(Channel > 4)
            return 0;
        setDiffChannal_ADC2(Channel);
        writeCmd(CMD_START2);
        Value = readADC2Data();
    }
    return Value;
}

/***************************************
* 函数: getAll
* 参数: List - 通道号数组, Value - 存放采样值的数组, Number - 数组长度
* 功能: 读取 ADC1 所有指定通道的数据
***************************************/
void ADS1263::getAll(UBYTE *List, UDOUBLE *Value, int Number)
{
    for(int i = 0; i < Number; i++) {
        Value[i] = getChannalValue(List[i]);
    }
}

/***************************************
* 函数: getAll_ADC2
* 参数: ADC_Value - 存放 ADC2 采样值的数组（假定至少10个）
* 功能: 读取 ADC2 所有通道的数据
***************************************/
void ADS1263::getAll_ADC2(UDOUBLE *ADC_Value)
{
    for(int i = 0; i < 10; i++) {
        ADC_Value[i] = getChannalValueADC2(i);
        writeCmd(CMD_STOP2);
    }
}


void ADS1263::softReset(void){
    writeCmd(CMD_RESET);
    DEV_Delay_ms(200);
}
void ADS1263::softStop(void){
    writeCmd(CMD_STOP1);
}
void ADS1263::softStart(void){
    writeCmd(CMD_START1);
}
void ADS1263::calibrate(void){
    // 1.Set to Continuous mode
    // # default is continuous mode
    // # 2.Select the desired gain and reference voltage of the ADC.
    UBYTE MODE2 = 0x00;    //0x80:PGA bypassed, 0x00:PGA enabled
    MODE2 |= (ADS1263_GAIN_32 << 4) | ADS1263_20SPS;
    writeReg(REG_MODE2, MODE2);
    if(readData(REG_MODE2) == MODE2){
        ROS_INFO("REG_MODE2 success");
    }
    else{
        ROS_INFO("REG_MODE2 unsuccess");
    }
    UBYTE REFMUX = 0x12;   //AIN2 - AIN3 as REF
    writeReg(REG_REFMUX, REFMUX);
    if(readData(REG_REFMUX) == REFMUX){
    ROS_INFO("REG_REFMUX success");
    }
    else{
    ROS_INFO("REG_REFMUX unsuccess");
    }
    // 3.Set the input channel
    UBYTE INPMUX = (0<<4) | 1;   // set to 0xFF to force open all input connections
    writeReg(REG_INPMUX, INPMUX);
    // 4.Start conversions
    writeCmd(CMD_START1);
    // 5.Send the calibration command
    writeCmd(CMD_SYOCAL1);
    DEV_Delay_ms(50);
    // 6.Wait for the calibration to complete
    while(DEV_GPIO_Read(DEV_DRDY_PIN) == DEV_GPIO_HIGH){
        ROS_INFO("DRDY_PIN is 1");
        DEV_Delay_ms(50);
    }
    ROS_INFO("Calibration Done");
    ROS_INFO("Offset: %d",readData(REG_OFCAL2)<<16|readData(REG_OFCAL1)<<8|readData(REG_OFCAL0));


}
void ADS1263::setGainRate(void){
    UDOUBLE MODE2 = 0x00;    // 0x80:PGA bypassed, 0x00:PGA enabled
    MODE2 |= (ADS1263_GAIN_32 << 4) | ADS1263_1200SPS;
    writeReg(REG_MODE2, MODE2);
    if(readData(REG_MODE2) == MODE2){
        ROS_INFO("REG_MODE2 success");
    }
    else{
        ROS_INFO("REG_MODE2 unsuccess");
    }
}
void ADS1263::setDelay(void){
    UBYTE MODE0 = ADS1263_DELAY_0s;
    writeReg(REG_MODE0, MODE0);
    if(readData(REG_MODE0) == MODE0){
        ROS_INFO("REG_MODE0 success");
    }
    else{
        ROS_INFO("REG_MODE0 unsuccess");
    }
}
void ADS1263::setFilter(void){
    UBYTE MODE1 = 0x60;    // Digital Filter; 0x84:FIR, 0x64:Sinc4, 0x44:Sinc3, 0x24:Sinc2, 0x04:Sinc1
    writeReg(REG_MODE1, MODE1);
    if(readData(REG_MODE1) == MODE1){
        ROS_INFO("REG_MODE1 success");
    }
    else{
        ROS_INFO("REG_MODE1 unsuccess");
    }
}
void ADS1263::setDelayPulseMode(void){
    UBYTE MODE0 = ADS1263_DELAY_0s;
    MODE0 = MODE0 | 0x40;
    writeReg(REG_MODE0, MODE0);
    if(readData(REG_MODE0)== MODE0){
        ROS_INFO("REG_MODE0 success");
    }
    else{
        ROS_INFO("REG_MODE0 unsuccess");
    }
}
void ADS1263::Init(void){
    softReset();
    calibrate();
    setDelayPulseMode();
    setGainRate();
    setFilter();
    setMode(1);
    setDiffChannal(0);
}
