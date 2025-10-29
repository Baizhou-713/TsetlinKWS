#include <stdio.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xgpiops.h"
#include "sleep.h"
#include "xtime_l.h"
#include "xgpio.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "tf_card.h"
#include "xspips.h"
#include "spi_config.h"
#include "xiicps.h"
#include "adau1761.h"

// Device ID
#define GPIOPS_ID           XPAR_XGPIOPS_0_DEVICE_ID
#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID
#define IIC_DEVICE_ID       XPAR_XIICPS_0_DEVICE_ID
#define AXI_GPIO_DEVICE_ID  XPAR_AXI_GPIO_0_DEVICE_ID
#define SCUGIC_ID           XPAR_SCUGIC_SINGLE_DEVICE_ID
#define AXI_GPIO_INT_ID     XPAR_FABRIC_GPIO_0_VEC_ID

#define PL_DONE_CHANNEL1    1
#define PL_DONE_CH1_MASK    XGPIO_IR_CH1_MASK
#define TMA_SPI_SELECT      0x00

#define EMIO_RESULT_0       54
#define EMIO_RESULT_1       55
#define EMIO_RESULT_2       56
#define EMIO_RESULT_3       57

#define WINDOW_SIZE                 40
#define FILL_SILENCE_MAX_CNT        40
#define DETECTING_CONS_RESULT_CNT   20

const char* label[] = {"yes", "no", "up", "down", "left", "right", "on", "off", "stop", "go", "silence", "unknown"};


/************************** Function declaration *****************************/
int setup_interrupt_system(XScuGic *gic_inst_ptr, XGpio *axi_gpio_inst_ptr, u16 AXI_GpioIntrId);
static void intr_handler(void *CallbackRef);
int initial_spi_system(XSpiPs *SpiInstancePtr, u16 SpiDeviceId);
void init_ADAU1761();
void ADAU1761_Write_Reg(u16 reg_addr, u16 length, u8 reg_data[]);


// Config pointer
XScuGic_Config *scugic_cfg_ptr;
XIicPs_Config *I2c_Config;

// Inst
XGpioPs gpiops_inst;
XGpio axi_gpio_inst;
XScuGic scugic_inst;
XSpiPs SpiInstance;
XIicPs Iic;

int inference_finish_flag = 0;

typedef enum {
    detecting,
    fill_silence
} sys_state;

sys_state current_state = detecting;

u8 print_flag = 0;
u8 fill_silence_cnt = 0;
u8 consecutive_time = 0;
u8 consecutive_result = 0;


int main()
{
    int status;
    int window_idx = 0;
    u8 result_window[WINDOW_SIZE];
    u8 result_bit[4];
    u8 result;
    u8 result_count[12] = {0};
    u8 max_result, max_result_count;
    u8 last_result;
    
    printf("Start Tsetlin Machine Accelerator for Keyword Spotting!\n\r");
    
    for (int i = 0; i < WINDOW_SIZE; i++) {
        result_window[i] = 10;
    }

    result_count[10] = WINDOW_SIZE;
    last_result = 10;
    
    XGpioPs_Config *gpiops_cfg_ptr;

    // Initial GPIO
    gpiops_cfg_ptr = XGpioPs_LookupConfig(GPIOPS_ID);
    XGpioPs_CfgInitialize(&gpiops_inst, gpiops_cfg_ptr, gpiops_cfg_ptr->BaseAddr);
    
    // Set EMIO_RESULT* as input
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_RESULT_0, 0);
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_RESULT_1, 0);
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_RESULT_2, 0);
    XGpioPs_SetDirectionPin(&gpiops_inst, EMIO_RESULT_3, 0);
    
    // Initial TF card
    status = sd_mount();
    if(status != XST_SUCCESS){
        xil_printf("Failed to open SD card!\n\r");
        return 0;
    } else{
        xil_printf("Success to open SD card!\n\r");
    }
    
    // Initial I2C
    I2c_Config = XIicPs_LookupConfig(IIC_DEVICE_ID);
    XIicPs_CfgInitialize(&Iic, I2c_Config, I2c_Config->BaseAddress);
    XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);    // Setting I2C speed
    
    init_ADAU1761();
    print("Configure ADAU1761 Finish!\n\r");
    
    // Initial Interrupt
    scugic_cfg_ptr = XScuGic_LookupConfig(SCUGIC_ID);
    XScuGic_CfgInitialize(&scugic_inst, scugic_cfg_ptr, scugic_cfg_ptr->CpuBaseAddress);
    
    // Initial PL AXI GPIO
    XGpio_Initialize(&axi_gpio_inst, AXI_GPIO_DEVICE_ID);
    status = setup_interrupt_system(&scugic_inst, &axi_gpio_inst, AXI_GPIO_INT_ID);
    if (status != XST_SUCCESS) {
        xil_printf("Setup interrupt system failed\r\n");
        return XST_FAILURE;
    } else {
        xil_printf("Setup interrupt system Finished!\r\n");
    }
    
    // READ DATA FROM TF CARD
    read_model_data();
    
    // initial SPI
    status = initial_spi_system(&SpiInstance, SPI_DEVICE_ID);
    if (status != XST_SUCCESS) {
		xil_printf("Setup SPI Failed!\r\n");
		return XST_FAILURE;
	} else {
        xil_printf("Setup SPI Finished!\r\n");
    }
    
    xil_printf("Loading TM model...\r\n");
    
    status = initial_TMA(&SpiInstance);
    if (status != XST_SUCCESS) {
		xil_printf("TsetlinKWS initialization Failed!\r\n");
		return XST_FAILURE;
	} else {
        xil_printf("TsetlinKWS initialization Finished!\r\n");
    }

    xil_printf("Please speak:{yes, no, up, down, left, right, on, off, stop, go}\r\n");

    int print_cnt = 0;
    while(1){

        if (inference_finish_flag == 1){
            inference_finish_flag = 0;
            
            // get result
            result_bit[0] = XGpioPs_ReadPin(&gpiops_inst, EMIO_RESULT_0);
            result_bit[1] = XGpioPs_ReadPin(&gpiops_inst, EMIO_RESULT_1);
            result_bit[2] = XGpioPs_ReadPin(&gpiops_inst, EMIO_RESULT_2);
            result_bit[3] = XGpioPs_ReadPin(&gpiops_inst, EMIO_RESULT_3);
            result = ((result_bit[3] << 3) | (result_bit[2] << 2) | (result_bit[1] << 1) | (result_bit[0] << 0));
            
            // pop one elements
            result_count[result_window[window_idx]] -= 1;
            
            // Only when two identical results are detected consecutively, update the element.
            if (result == last_result) {
                // update elements
                result_window[window_idx] = result;
            } else {
                result_window[window_idx] = 10;
            }

            // Fill in the "silence" result within the waiting time after a trigger to prevent "unknown" false alarms.
            if (current_state == fill_silence) {
                result_window[window_idx] = 10;
            }
            
            // update count
            result_count[result_window[window_idx]] += 1;
            
            // update index
            window_idx += 1;
            if (window_idx == WINDOW_SIZE){
                window_idx = 0;
            }
            
            // calculate average result
            max_result = 0;
            max_result_count = 0;
            for (int i = 0; i < 12; i++){
                if (result_count[i] > max_result_count){
                    max_result = i;
                    max_result_count = result_count[i];
                }
            }
            
            int start_idx;

            start_idx = window_idx;
            consecutive_time = 0;
            consecutive_result = result_window[window_idx];
            for (int i = 0; i < WINDOW_SIZE; i++) {

                if (consecutive_result == result_window[start_idx]) {
                    consecutive_time += 1;
                } else {
                    // reset timer
                    consecutive_result = result_window[start_idx];
                    consecutive_time = 0;
                }

                if (consecutive_time == DETECTING_CONS_RESULT_CNT && consecutive_result != 10 && current_state == detecting){
                    print_flag = 1;
                    break;
                }

                start_idx ++;
                if (start_idx == WINDOW_SIZE) {
                    start_idx = 0;
                }
            }
            
            
            switch (current_state){
                case detecting:
                    fill_silence_cnt = 0;
                    if (print_flag) {
                        printf("yes:%d, no:%d, up:%d, down:%d, left:%d, right:%d, on:%d, off:%d, stop:%d, go:%d, silence:%d, unknown:%d\r\n",
                                result_count[0], result_count[1], result_count[2], result_count[3], result_count[4], result_count[5],
                                result_count[6], result_count[7], result_count[8], result_count[9], result_count[10], result_count[11]);

                        printf("Detect: %s\n\r", label[consecutive_result]);
                    }

                    break;
                case fill_silence:
                    fill_silence_cnt += 1;
                    print_flag = 0;

                    break;
            }

            switch (current_state){
                case detecting:
                    if (print_flag == 1)
                        current_state = fill_silence;
                    break;
                case fill_silence:
                    if (fill_silence_cnt == FILL_SILENCE_MAX_CNT)
                        current_state = detecting;
                    break;
                default:current_state = detecting; break;
            }
            
            last_result = result;
        }
    }
}

int initial_spi_system(XSpiPs *SpiInstancePtr, u16 SpiDeviceId)
{
    int Status;
    XSpiPs_Config *SpiConfig;
    
    // Initial SPI
	SpiConfig = XSpiPs_LookupConfig(SpiDeviceId);
	Status = XSpiPs_CfgInitialize(SpiInstancePtr, SpiConfig, SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
        xil_printf("Setup SPI failed\r\n");
		return XST_FAILURE;
	}

	// Perform a self-test to check hardware build
	Status = XSpiPs_SelfTest(SpiInstancePtr);
	if (Status != XST_SUCCESS) {
        xil_printf("SPI SelfTest failed\r\n");
		return XST_FAILURE;
	}

	// Set the Spi device as a master. External loopback is required.
    //XSPIPS_MASTER_OPTION          : master
    //XSPIPS_FORCE_SSELECT_OPTION   : manual CS

	XSpiPs_SetOptions(SpiInstancePtr, XSPIPS_MASTER_OPTION |
			   XSPIPS_FORCE_SSELECT_OPTION);

	XSpiPs_SetClkPrescaler(SpiInstancePtr, XSPIPS_CLK_PRESCALE_256);
    
    // Enable CS
    XSpiPs_SetSlaveSelect(SpiInstancePtr, TMA_SPI_SELECT);
    
    return XST_SUCCESS;
}

int setup_interrupt_system(XScuGic *gic_inst_ptr, XGpio *axi_gpio_inst_ptr, u16 AXI_GpioIntrId)
{
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                    (Xil_ExceptionHandler) XScuGic_InterruptHandler, gic_inst_ptr);
    Xil_ExceptionEnable();

    XScuGic_SetPriorityTriggerType(gic_inst_ptr, AXI_GpioIntrId, 0xA0, 0x03);
    
    XScuGic_Connect(gic_inst_ptr, AXI_GpioIntrId,
                    (Xil_ExceptionHandler) intr_handler, (void *) axi_gpio_inst_ptr);

    XScuGic_Enable(gic_inst_ptr, AXI_GpioIntrId);

    XGpio_SetDataDirection(axi_gpio_inst_ptr, PL_DONE_CHANNEL1, 1);
    XGpio_InterruptEnable(axi_gpio_inst_ptr, PL_DONE_CH1_MASK);
    XGpio_InterruptGlobalEnable(axi_gpio_inst_ptr);

    return XST_SUCCESS;

}

void intr_handler(void *CallbackRef)
{

    XGpio *GpioPtr = (XGpio *)CallbackRef;
    usleep(0.3);
    if (XGpio_DiscreteRead(GpioPtr, PL_DONE_CHANNEL1) == 1) {
        XGpio_InterruptDisable(GpioPtr, PL_DONE_CH1_MASK);
        inference_finish_flag = 1;
    }

    XGpio_InterruptClear(GpioPtr, PL_DONE_CH1_MASK);
    XGpio_InterruptEnable(GpioPtr, PL_DONE_CH1_MASK);
}


void init_ADAU1761(){
    
    ADAU1761_Write_Reg( REG_CLKCTRLREGISTER_ADAU1761_ADDR, REG_CLKCTRLREGISTER_ADAU1761_BYTE, R0_CLKCTRLREGISTER_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_PLLCRLREGISTER_ADAU1761_ADDR, REG_PLLCRLREGISTER_ADAU1761_BYTE, R1_PLLCRLREGISTER_ADAU1761_Default );
	//SIGMA_WRITE_DELAY         ( R2_DELAY_ADAU1761_SIZE, R2_DELAY_ADAU1761_Default );
	sleep(1);
    ADAU1761_Write_Reg( REG_SERIAL_PORT_CONTROL_0_ADAU1761_ADDR , R3_SERIAL_PORT_CONTROL_REGISTERS_ADAU1761_SIZE, R3_SERIAL_PORT_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_ALC_CONTROL_0_ADAU1761_ADDR , R4_ALC_CONTROL_REGISTERS_ADAU1761_SIZE, R4_ALC_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_MICCTRLREGISTER_ADAU1761_ADDR, REG_MICCTRLREGISTER_ADAU1761_BYTE, R5_MICCTRLREGISTER_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_RECORD_PWR_MANAGEMENT_ADAU1761_ADDR , R6_RECORD_INPUT_SIGNAL_PATH_REGISTERS_ADAU1761_SIZE, R6_RECORD_INPUT_SIGNAL_PATH_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_ADC_CONTROL_0_ADAU1761_ADDR , R7_ADC_CONTROL_REGISTERS_ADAU1761_SIZE, R7_ADC_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_PLAYBACK_MIXER_LEFT_CONTROL_0_ADAU1761_ADDR , R8_PLAYBACK_OUTPUT_SIGNAL_PATH_REGISTERS_ADAU1761_SIZE, R8_PLAYBACK_OUTPUT_SIGNAL_PATH_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_CONVERTER_CTRL_0_ADAU1761_ADDR , R9_CONVERTER_CONTROL_REGISTERS_ADAU1761_SIZE, R9_CONVERTER_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_DAC_CONTROL_0_ADAU1761_ADDR , R10_DAC_CONTROL_REGISTERS_ADAU1761_SIZE, R10_DAC_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_SERIAL_PORT_PAD_CONTROL_0_ADAU1761_ADDR , R11_SERIAL_PORT_PAD_CONTROL_REGISTERS_ADAU1761_SIZE, R11_SERIAL_PORT_PAD_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_COMM_PORT_PAD_CTRL_0_ADAU1761_ADDR , R12_COMMUNICATION_PORT_PAD_CONTROL_REGISTERS_ADAU1761_SIZE, R12_COMMUNICATION_PORT_PAD_CONTROL_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_JACKREGISTER_ADAU1761_ADDR, REG_JACKREGISTER_ADAU1761_BYTE, R13_JACKREGISTER_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_DSP_ENABLE_REGISTER_ADAU1761_ADDR, REG_DSP_ENABLE_REGISTER_ADAU1761_BYTE, R14_DSP_ENABLE_REGISTER_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_CRC_IDEAL_1_ADAU1761_ADDR , R15_CRC_REGISTERS_ADAU1761_SIZE, R15_CRC_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_GPIO_0_CONTROL_ADAU1761_ADDR , R16_GPIO_REGISTERS_ADAU1761_SIZE, R16_GPIO_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_NON_MODULO_RAM_1_ADAU1761_ADDR , R17_NON_MODULO_REGISTERS_ADAU1761_SIZE, R17_NON_MODULO_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_WATCHDOG_ENABLE_ADAU1761_ADDR , R18_WATCHDOG_REGISTERS_ADAU1761_SIZE, R18_WATCHDOG_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_SAMPLE_RATE_SETTING_ADAU1761_ADDR, REG_SAMPLE_RATE_SETTING_ADAU1761_BYTE, R19_SAMPLE_RATE_SETTING_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_ROUTING_MATRIX_INPUTS_ADAU1761_ADDR, REG_ROUTING_MATRIX_INPUTS_ADAU1761_BYTE, R20_ROUTING_MATRIX_INPUTS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_ROUTING_MATRIX_OUTPUTS_ADAU1761_ADDR, REG_ROUTING_MATRIX_OUTPUTS_ADAU1761_BYTE, R21_ROUTING_MATRIX_OUTPUTS_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_SERIAL_DATAGPIO_PIN_CONFIG_ADAU1761_ADDR, REG_SERIAL_DATAGPIO_PIN_CONFIG_ADAU1761_BYTE, R22_SERIAL_DATAGPIO_PIN_CONFIG_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_DSP_SLEW_MODES_ADAU1761_ADDR, REG_DSP_SLEW_MODES_ADAU1761_BYTE, R23_DSP_SLEW_MODES_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_SERIAL_PORT_SAMPLE_RATE_SETTING_ADAU1761_ADDR, REG_SERIAL_PORT_SAMPLE_RATE_SETTING_ADAU1761_BYTE, R24_SERIAL_PORT_SAMPLE_RATE_SETTING_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_CLOCK_ENABLE_REG_0_ADAU1761_ADDR , R25_CLOCK_ENABLE_REGISTERS_ADAU1761_SIZE, R25_CLOCK_ENABLE_REGISTERS_ADAU1761_Default );
	ADAU1761_Write_Reg( PROGRAM_ADDR_ADAU1761, PROGRAM_SIZE_ADAU1761, Program_Data_ADAU1761 );
	ADAU1761_Write_Reg( PARAM_ADDR_ADAU1761, PARAM_SIZE_ADAU1761, Param_Data_ADAU1761 );
	ADAU1761_Write_Reg( REG_SAMPLE_RATE_SETTING_ADAU1761_ADDR, REG_SAMPLE_RATE_SETTING_ADAU1761_BYTE, R28_SAMPLE_RATE_SETTING_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_DSP_RUN_REGISTER_ADAU1761_ADDR, REG_DSP_RUN_REGISTER_ADAU1761_BYTE, R29_DSP_RUN_REGISTER_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_DEJITTER_REGISTER_CONTROL_ADAU1761_ADDR, REG_DEJITTER_REGISTER_CONTROL_ADAU1761_BYTE, R30_DEJITTER_REGISTER_CONTROL_ADAU1761_Default );
	ADAU1761_Write_Reg( REG_DEJITTER_REGISTER_CONTROL_ADAU1761_ADDR, REG_DEJITTER_REGISTER_CONTROL_ADAU1761_BYTE, R31_DEJITTER_REGISTER_CONTROL_ADAU1761_Default );
}



void ADAU1761_Write_Reg(u16 reg_addr, u16 length, u8 reg_data[])
{
    u8 I2C_DATA[MAX_BRUST_LEN+2];
    
    if (length > MAX_BRUST_LEN){
        print("IIC REG Address write length exceed MAX_BRUST_LEN");
    }
    
    // register address
    I2C_DATA[0] = reg_addr >> 8;    // High byte
    I2C_DATA[1] = reg_addr & 0xFF;  // Low byte
    
    // data
    for (u16 i = 0; i < length; i++){
        I2C_DATA[2+i] = reg_data[i];
    }
    
    // I2C write
    XIicPs_MasterSendPolled(&Iic, I2C_DATA, length + 2, ADAU1761_DEV_ADDR);
    
    // wait for finish
    while (XIicPs_BusIsBusy(&Iic));
}
