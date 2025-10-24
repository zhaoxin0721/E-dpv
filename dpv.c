#define DEBUG

#include "dpv.h"
#include "../drivers/freistat_ramp.h"
#include "../drivers/ad594x.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdlib.h>

// DPV辅助函数声明
static void dpv_set_voltage(float voltage_mV);
static void dpv_measure_current(float *current_uA);
static void dpv_init_afe(void);
static AD5940Err dpv_rtia_calibration(void);

LOG_MODULE_REGISTER(elec_dpv, LOG_LEVEL_INF);

// 基于custom_cv.c的常量定义（移除重复定义，使用freistat_ramp.h中的定义）
#define LPTIARTIA_SEL LPTIARTIA_10K
#define ADCPGA_Gain   ADCPGA_1P5
#define ADCRef_Volt   1820.0f
#define ADCSinc3OSR   ADCSINC3OSR_4
#define ADC_Clk_Freq  (16000000.0f)
#define R_Cal_Val     (10000.0f)  // 10kΩ RTIA

// RTIA校准值
static float rtia_calibrated_value = R_Cal_Val;

void elec_dpv_loop();
static volatile bool working = false;
static elec_dpv_config_t config = {
  .equilibration_time = 2000.0f,    // 平衡时间：2000ms
  .start_voltage      = -500.0f,    // 起始电位：-500mV
  .end_voltage        = 500.0f,     // 终止电位：500mV
  .step_height        = 10.0f,      // 电位步长：10mV
  .pulse_amplitude    = 200.0f,     // 脉冲电位：200mV
  .pulse_width        = 20.0f,      // 脉冲时间：20ms
  .scan_rate          = 100.0f,     // 扫描速率：100mV/s
  .sample_delay       = 10.0f,      // 脉冲后采样延迟：10ms
  .rtia               = LPTIARTIA_8K,
};
static uint32_t index;

int elec_dpv_init()
{
  return 0;
}

int elec_dpv_config(elec_dpv_config_t config_)
{
  config.equilibration_time = config_.equilibration_time;
  config.start_voltage      = config_.start_voltage;
  config.end_voltage        = config_.end_voltage;
  config.step_height        = config_.step_height;
  config.pulse_amplitude    = config_.pulse_amplitude;
  config.pulse_width        = config_.pulse_width;
  config.scan_rate          = config_.scan_rate;
  config.sample_delay       = config_.sample_delay;
  config.rtia               = config_.rtia;
  
  LOG_INF("DPV配置更新: 起始%.1fmV, 终止%.1fmV, 步长%.1fmV, 脉冲%.1fmV, 宽度%.1fms",
          config.start_voltage, config.end_voltage, config.step_height, 
          config.pulse_amplitude, config.pulse_width);
  
  // 初始化AFE配置
  dpv_init_afe();
  
  return 0;
}

int elec_dpv_start()
{
  index = 0;
  working = true;
  return elec_method_commit(&elec_dpv_loop);
}

int elec_dpv_stop()
{
  index = 0;
  working = false;
  int err = elec_method_commit(NULL);
  LOG_INF("DPV stop: still working? %s", working ? "yes" : "no");
  return err;
}

bool elec_dpv_is_working()
{
  return working;
}

// Implementation:
static float RampLFOSCFreq;                /* Measured LFOSC frequency */

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize(); /* Call this right after AFE reset */
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  /* Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bTRUE; /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB; /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3; /* */
  fifo_cfg.FIFOThresh = 4;          /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB; /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0 | AFEINTSRC_CUSTOMINT1 | AFEINTSRC_GPT1INT_TRYBRK | AFEINTSRC_DATAFIFOOF, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIO */
  gpio_cfg.FuncSet = GP0_INT | GP1_GPIO | GP2_SYNC; /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
  gpio_cfg.OutVal = AGPIO_Pin1; //set high to turn off LED
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;        /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;              /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &RampLFOSCFreq);
#ifdef DEBUG
  printk("Measured LFOSC Freq (used for sequencer timing) - rounded: %d Hz\n", (int)(RampLFOSCFreq + 0.5));
#endif
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /*  */
  return 0;
}

void elec_dpv_loop()
{
  // 配置AFE - 基于cv.c的成功实现
  AD5940PlatformCfg();
  
  // 执行平衡时间
  LOG_INF("进入平衡阶段，持续 %.0f ms...", config.equilibration_time);
    k_msleep((uint32_t)config.equilibration_time);

  // 初始化AFE - 基于custom_cv.c
    dpv_init_afe();

  // // 计算DPV扫描参数
  // int dir = (config.end_voltage >= config.start_voltage) ? 1 : -1;
  // int total_steps = (int)((config.end_voltage - config.start_voltage) / config.step_height * dir);
  
    // 计算DPV扫描参数（用绝对差/步长，支持正/负扫）
  float span = config.end_voltage - config.start_voltage;
  int dir = (span >= 0.0f) ? 1 : -1;
  int total_steps = (int)(fabs(span) / fabs(config.step_height)) + 1;
  if (total_steps <= 0) total_steps = 1;

  /* 每步积分时间 t_int (ms) = |E_step| / scan_rate ; 保证 t_int >= pulse + sample_delay */
  float t_int_ms = 1000.0f;
  if (config.scan_rate > 0.0f) {
    t_int_ms = fabs(config.step_height) / config.scan_rate * 1000.0f;
    if (t_int_ms < ((float)config.pulse_width + (float)config.sample_delay + 1.0f))
      t_int_ms = (float)config.pulse_width + (float)config.sample_delay + 1.0f;
  }
  // LOG_INF("DPV扫描:起始电位 %.1f mV,终止电位 %.1f mV,步长 %.1f mV,脉冲 %.1f mV,共 %d 步", 
  //         config.start_voltage, config.end_voltage, config.step_height, config.pulse_amplitude, total_steps);
  LOG_INF("DPV扫描: 起始 %.1f mV -> 终止 %.1f mV, 步长 %.1f mV, 脉冲 %.1f mV, 步数 %d, t_int=%.1fms",
          config.start_voltage, config.end_voltage, config.step_height, config.pulse_amplitude, total_steps, t_int_ms);
 

  // // DPV主循环：严格按照理论图实现
  // for (int step = 0; step < total_steps && working; step++) {
  //   // 计算当前基础电位
  //   float base_voltage = config.start_voltage + step * config.step_height * dir;
    
  //   // // 钳制电位范围
  //   // if (dir > 0) {
  //   //   if (base_voltage > config.end_voltage) base_voltage = config.end_voltage;
  //   //   if (base_voltage < config.start_voltage) base_voltage = config.start_voltage;
  //   // } else {
  //   //   if (base_voltage < config.end_voltage) base_voltage = config.end_voltage;
  //   //   if (base_voltage > config.start_voltage) base_voltage = config.start_voltage;
  //   // }
    
  //   // 步骤1：设置基础电位并等待稳定
  //   dpv_set_voltage(base_voltage);
  //   k_msleep(200); // 增加等待时间确保电压稳定
    
  //   // 步骤2：测量脉冲前电流 (i1)
  //   float current_before = 0.0f;
  //   dpv_measure_current(&current_before);
    
  //   // 步骤3：应用脉冲电位
  //   float pulse_voltage = base_voltage + config.pulse_amplitude;
  //   dpv_set_voltage(pulse_voltage);
    
  //   // 步骤4：等待脉冲宽度
  //   k_msleep((uint32_t)config.pulse_width + 100); // 增加脉冲等待时间
    
  //   // 步骤5：测量脉冲后电流 (i2)
  //   float current_after = 0.0f;
  //   dpv_measure_current(&current_after);
  for (int step = 0; step < total_steps && working; step++) {
    /* 计算当前基线电位（按 step 顺序）*/
    float base_voltage = config.start_voltage + (float)step * config.step_height * (float)dir;

    /* 将脉冲放在步末：pre_wait = t_int - pulse - sample_delay */
   float pre_wait_ms = t_int_ms - (float)config.pulse_width - (float)config.sample_delay;
   if (pre_wait_ms < 0.0f) pre_wait_ms = 0.0f;

    /* 步骤1：设置基线电位并等待到脉冲前 */
    dpv_set_voltage(base_voltage);
    if (pre_wait_ms > 0.0f) k_msleep((uint32_t)pre_wait_ms);
    k_msleep(5); /* 小的稳定等待，靠 ADC 过滤进一步稳定 */

    /* 步骤2：在脉冲前采样 i1 */
    float current_before = 0.0f;
    dpv_measure_current(&current_before);

    /* 步骤3：施加脉冲（脉冲在步末）*/
    float pulse_voltage = base_voltage + config.pulse_amplitude;
    dpv_set_voltage(pulse_voltage);
    k_msleep((uint32_t)config.pulse_width);

    /* 步骤4：脉冲后等待采样延时并测 i2 */
    k_msleep((uint32_t)config.sample_delay);
    float current_after = 0.0f;
    dpv_measure_current(&current_after);

    /* 恢复基线（为下一步准备） */
    dpv_set_voltage(base_voltage);
    k_msleep(5);
    // 步骤6：计算差分电流 (i2 - i1)
    float diff_current = current_after - current_before;
    
    LOG_DBG("Step %d: base %.1f mV -> pulse %.1f mV, i1=%.3f uA, i2=%.3f uA, diff=%.3f uA", 
            step, base_voltage, pulse_voltage, current_before, current_after, diff_current);
    
    // 发送数据
    uint8_t data[10];
    uint16_t id = step;
    int32_t uV = (int32_t)(base_voltage * 1000); // 基础电位 -> uV
    int32_t nA = (int32_t)(diff_current * 1000); // 差分电流 -> nA
    
    data[0] = (uint8_t)((id >> 8) & 0xFF);
    data[1] = (uint8_t)(id & 0xFF);
    data[2] = (uint8_t)((nA >> 24) & 0xFF);
    data[3] = (uint8_t)((nA >> 16) & 0xFF);
    data[4] = (uint8_t)((nA >> 8) & 0xFF);
    data[5] = (uint8_t)((nA) & 0xFF);
    data[6] = (uint8_t)((uV >> 24) & 0xFF);
    data[7] = (uint8_t)((uV >> 16) & 0xFF);
    data[8] = (uint8_t)((uV >> 8) & 0xFF);
    data[9] = (uint8_t)((uV) & 0xFF);
    
    send_fac_stream_directly(data, 10);
    
    // 详细调试信息
    LOG_INF("DPV Step %d: base %.1f mV, pulse %.1f mV, before %.3f uA, after %.3f uA, diff %.3f uA", 
            step, base_voltage, pulse_voltage, current_before, current_after, diff_current);
    
    // 检查差分电流是否合理
    if (abs(diff_current) < 0.001f) {
      LOG_WRN("Warning: Differential current is very small: %.6f uA", diff_current);
    }
    
    // 等待采样延迟
    k_msleep((uint32_t)config.sample_delay);
  }

  // 发送停止信号
  uint8_t stop_signal[2];
  stop_signal[0] = 0xFF;
  stop_signal[1] = 0xFF;
  send_fac_stream_directly(stop_signal, 2);
  
  elec_dpv_stop();
}

// DPV辅助函数实现 - 基于custom_cv.c的成功实现
static void dpv_set_voltage(float voltage_mV)
{
  // 基于custom_cv.c的正确DAC设置
  uint32_t VbiasCode, VzeroCode;
  
  // Vzero设置为1.1V基准
  VzeroCode = (uint32_t)(1100.0f / DAC6BITVOLT_1LSB);
  
  // Vbias是相对于Vzero的偏移电压
  // 注意：Vbias需要转换为相对于1.1V的偏移
  float voltage_offset = voltage_mV + 1100.0f; // 转换为绝对电压
  VbiasCode = (uint32_t)(voltage_offset / DAC12BITVOLT_1LSB);
  
  // 限制DAC代码范围
  if (VbiasCode > 4095) VbiasCode = 4095;
  if (VzeroCode > 63) VzeroCode = 63;
  
  // 组合DAC数据：Vzero(6位) + Vbias(12位)
  uint32_t dac_data = (VzeroCode << 12) | VbiasCode;
  
  // 写入DAC寄存器
  AD5940_WriteReg(REG_AFE_LPDACDAT0, dac_data);
  
  // 等待DAC稳定
  k_usleep(100);
  
  LOG_DBG("Set voltage: %.1f mV, VzeroCode: %d, VbiasCode: %d, DAC: 0x%08X", 
          voltage_mV, VzeroCode, VbiasCode, dac_data);
}

static void dpv_measure_current(float *current_uA)
{
  // 基于custom_cv.c的ccv_adc_rtia_read函数实现
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);                       /* Start ADC */
  k_usleep(250);                                                /* wait 250us for reference power up */
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);                       /* Start ADC conversion */
  
  // 等待转换完成
  uint32_t timeout = 1000; // 1秒超时
  while (!AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_SINC2RDY) && timeout > 0) {
    k_usleep(10);
    timeout--;
  }
  
  if (timeout > 0) {
    // 读取ADC结果 - 使用SINC3滤波器
    uint32_t code = AD5940_ReadAfeResult(AFERESULT_SINC3);
    
    // 转换为电压 - 注意负号！基于custom_cv.c
    float volt = -AD5940_ADCCode2Volt(code, ADCPGA_Gain, ADCRef_Volt);
    
    // 转换为电流 - 使用校准的RTIA值
    *current_uA = volt / rtia_calibrated_value * 1e3f;
    
    // LOG_DBG("Current measurement: ADC=0x%08X, Voltage=%.3f mV, Current=%.3f uA, RTIA=%.1f", 
    //         code, volt, *current_uA, rtia_calibrated_value);
    LOG_INF("Current measurement: ADC=0x%08X, Voltage=%.3f mV, Current=%.3f uA, RTIA=%.1f", 
            code, volt, *current_uA, rtia_calibrated_value);            
  } else {
    *current_uA = 0.0f; // 超时，返回0
    LOG_WRN("Current measurement timeout");
  }
  
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);       /* Stop ADC */
}

static AD5940Err dpv_rtia_calibration(void)
{
  // 基于custom_cv.c的rtia_calibration函数
  fImpPol_Type RtiaCalValue;  /* Calibration result */
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE;                /* Magnitude + Phase */
  lprtia_cal.AdcClkFreq = ADC_Clk_Freq;  
  lprtia_cal.SysClkFreq = 16000000.0f;            /* System clock is 16MHz by default */
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;        /* Use SINC2 data as DFT data source */
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;         /* Maximum DFT number */
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = ADC_Clk_Freq / 4 / 22 / 2048 * 3; /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
  lprtia_cal.fRcal = R_Cal_Val;
  lprtia_cal.LpTiaRtia = LPTIARTIA_SEL;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  rtia_calibrated_value = RtiaCalValue.Magnitude;
  
  LOG_INF("RTIA calibration: %.3f Ω", rtia_calibrated_value);
  return AD5940ERR_OK;
}

static void dpv_init_afe(void)
{
  LOG_INF("Initializing DPV AFE based on custom_cv.c...");
  
  // 基于custom_cv.c的ccv_init函数实现
  
  // 1. 平台初始化
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */

  // 2. 时钟配置
  CLKCfg_Type clk_cfg;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);

  // 3. 中断控制器配置
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  // 4. GPIO配置
  AGPIOCfg_Type gpio_cfg;
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  
  // 5. LFOSC频率测量
  LFOSCMeasure_Type LfoscMeasure;
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  float LfoscFreq;
  AD5940_LFOSCMeasure(&LfoscMeasure, &LfoscFreq);
  
  // 6. AFE控制初始化
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */

  // 7. 参考电压配置
  AFERefCfg_Type aferef_cfg;
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);
  
  // 8. LPLoop配置
  LPLoopCfg_Type lploop_cfg;
  lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_BOOST3;
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
  lploop_cfg.LpAmpCfg.LpTiaRtia = LPTIARTIA_SEL;
  lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9);
  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lploop_cfg.LpDacCfg.DacData12Bit = 0x800;
  lploop_cfg.LpDacCfg.DacData6Bit = 0;
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA | LPDACSW_VZERO2LPTIA;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Step Vbias. Use 12bit DAC ouput */
  lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Base is Vzero. Use 6 bit DAC ouput */
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);

  // 9. DSP配置
  DSPCfg_Type dsp_cfg;
  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_Gain;

  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = ADCSinc3OSR;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;  /* ADC runs at 16MHz clock in this example, sample rate is 800kHz */
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;        /* We use data from SINC3 filter */
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_22;  /* Don't care */
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;   /* Don't care because it's disabled */
  AD5940_DSPCfgS(&dsp_cfg);

  // 10. RTIA校准 - 使用固定值而不是校准
  // dpv_rtia_calibration(); // 暂时禁用校准，使用固定值
  rtia_calibrated_value = 10000.0f; // 使用10kΩ固定值
  
  LOG_INF("DPV AFE initialization complete, using fixed RTIA: %.1f Ω", rtia_calibrated_value);
}
