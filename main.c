/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4 Emulated EEPROM example
*              for ModusToolbox.
*
* Related Document: See README.md 
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_em_eeprom.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define LED_DELAY_MS                (500u)

/* Logical Size of Emulated EEPROM in bytes */
#define LOGICAL_EM_EEPROM_SIZE      (15u)
#define LOGICAL_EM_EEPROM_START     (0u)

/* Location of reset counter in Emulated EEPROM */
#define RESET_COUNT_LOCATION        (13u)
/* Size of reset counter in bytes */
#define RESET_COUNT_SIZE            (2u)

/* ASCII "9" */
#define ASCII_NINE                  (0x39)
/* ASCII "0" */
#define ASCII_ZERO                  (0x30)
/* ASCII "P" */
#define ASCII_P                     (0x50)

/* Emulated EEPROM Configuration details. All the sizes mentioned are in bytes.
 * For details on how to configure these values refer to cy_em_eeprom.h. The
 * middleware documentation is provided in Emulated EEPROM API Reference Manual. 
 * The user can access it from the Documentation section in the Quick Panel.
 */
#define EM_EEPROM_SIZE              (256u)
#define BLOCKING_WRITE              (1u)
#define REDUNDANT_COPY              (1u)
#define WEAR_LEVELLING_FACTOR       (2u)
#define SIMPLE_MODE                 (0u)

#define EM_EEPROM_PHYSICAL_SIZE     (CY_EM_EEPROM_GET_PHYSICAL_SIZE(EM_EEPROM_SIZE, SIMPLE_MODE, WEAR_LEVELLING_FACTOR, REDUNDANT_COPY))

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void handle_error(uint32_t status, char *message);
void uart_print(char *message);

/*******************************************************************************
* Global Variables
********************************************************************************/
cy_stc_scb_uart_context_t CYBSP_UART_context;

cy_stc_eeprom_context_t em_eeprom_context;

/* Emulated EEPROM configuration and context structure. */
cy_stc_eeprom_config_t em_eeprom_config =
{
    .eepromSize         = EM_EEPROM_SIZE,           /* 256 bytes */
    .blockingWrite      = BLOCKING_WRITE,           /* Blocking writes enabled */
    .redundantCopy      = REDUNDANT_COPY,           /* Redundant copy enabled */
    .wearLevelingFactor = WEAR_LEVELLING_FACTOR,    /* Wear levelling factor of 2 */
    .simpleMode         = SIMPLE_MODE,              /* Simple mode disabled */
};

/* EEPROM storage Emulated EEPROM flash. */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)
const uint8_t em_eeprom_storage[EM_EEPROM_PHYSICAL_SIZE] = {0u};

/* RAM arrays for holding Emulated EEPROM read and write data respectively. */
uint8_t em_eeprom_read_array[LOGICAL_EM_EEPROM_SIZE];
uint8_t em_eeprom_write_array[LOGICAL_EM_EEPROM_SIZE] = { 0x50, 0x6F, 0x77, 0x65, 0x72, 0x20, 0x43, 0x79, 0x63, 0x6C, 0x65, 0x23, 0x20, 0x30, 0x30};
                                                 /* P     o     w     e     r           C     y     c     l     e     #           0     0 */

/********************************************************************************
* Function Name: main
*********************************************************************************
* Summary:
* The main function performs the following actions:
*  1. Initializes the BSP
*  2. Calls the functions to configure and initialize retarget IO and Emulated
*     EEPROM
*  3. Reads the Emulated EEPROM content if available
*  4. Increments the Emulated EEPROM content by one and writes the new content
*     back to Emulated EEPROM
*  5. Toggles the LED once every 0.5 seconds if the Emulated EEPROM read and 
*     write operations were successful
*
********************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_em_eeprom_status_t em_eeprom_status;
    cy_en_scb_uart_status_t uart_status;
    int count;
    char em_eeprom_char;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the UART */
    uart_status = Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    if(uart_status != CY_SCB_UART_SUCCESS)
    {
        /* UART initialization failed */
        CY_ASSERT(0);
    }
    
    /* Enable UART */
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    uart_print("\x1b[2J\x1b[;H");
    uart_print("***************************************************************\r\n");
    uart_print("PSoC 4 MCU: Emulated EEPROM example\r\n");
    uart_print("***************************************************************\r\n\n");

    /* Initialize the flash start address in Emulated EEPROM configuration 
     * structure
     */
    em_eeprom_config.userFlashStartAddr = (uint32_t) em_eeprom_storage;

    /* Initialize Emulated EEPROM */
    em_eeprom_status = Cy_Em_EEPROM_Init(&em_eeprom_config, &em_eeprom_context);
    handle_error(em_eeprom_status, "Emulated EEPROM Initialization Error \r\n");

    /* Read 15 bytes out of Emulated EEPROM memory */
    em_eeprom_status = Cy_Em_EEPROM_Read(LOGICAL_EM_EEPROM_START, em_eeprom_read_array,
                                            LOGICAL_EM_EEPROM_SIZE, &em_eeprom_context);
    handle_error(em_eeprom_status, "Emulated EEPROM Read failed \r\n");

    /* If first byte of Emulated EEPROM is not 'P', then write the data for initializing
     * the Emulated EEPROM content
     */
    if(ASCII_P != em_eeprom_read_array[0])
    {
        /* Erase Emulated EEPROM */
        em_eeprom_status = Cy_Em_EEPROM_Erase(&em_eeprom_context);
        handle_error(em_eeprom_status, "Emulated EEPROM erase failed \r\n");
        
        /* Write initial data to Emulated EEPROM. */
        em_eeprom_status = Cy_Em_EEPROM_Write(LOGICAL_EM_EEPROM_START,
                                                 em_eeprom_write_array,
                                                 LOGICAL_EM_EEPROM_SIZE,
                                                 &em_eeprom_context);
        handle_error(em_eeprom_status, "Emulated EEPROM Write failed \r\n");
    }
    else
    {
        /* The Emulated EEPROM content is valid. Increment Counter by 1 */
        em_eeprom_read_array[RESET_COUNT_LOCATION + 1]++;

        /* Counter is in ASCII, so handle overflow */
        if(em_eeprom_read_array[RESET_COUNT_LOCATION + 1] > ASCII_NINE)
        {
            /* Set lower digit to zero */
            em_eeprom_read_array[RESET_COUNT_LOCATION + 1] = ASCII_ZERO;

            /* Increment upper digit */
            em_eeprom_read_array[RESET_COUNT_LOCATION]++;

            /* Only increment to 99 */
            if(em_eeprom_read_array[RESET_COUNT_LOCATION] > ASCII_NINE)
            {
                em_eeprom_read_array[RESET_COUNT_LOCATION] = ASCII_NINE;
                em_eeprom_read_array[RESET_COUNT_LOCATION + 1] = ASCII_NINE;
            }
        }

        /* Only update the two count values in the Emulated EEPROM */
        em_eeprom_status = Cy_Em_EEPROM_Write(RESET_COUNT_LOCATION,
                                                 &em_eeprom_read_array[RESET_COUNT_LOCATION],
                                                 RESET_COUNT_SIZE,
                                                 &em_eeprom_context);
        handle_error(em_eeprom_status, "Emulated EEPROM Write failed \r\n");
    }

    /* Read contents of Emulated EEPROM after write */
    em_eeprom_status = Cy_Em_EEPROM_Read(LOGICAL_EM_EEPROM_START,
                                            em_eeprom_read_array, LOGICAL_EM_EEPROM_SIZE,
                                            &em_eeprom_context);
    handle_error(em_eeprom_status, "Emulated EEPROM Read failed \r\n" );

    for(count = 0; count < LOGICAL_EM_EEPROM_SIZE; count++)
    {
        em_eeprom_char = em_eeprom_read_array[count];
        Cy_SCB_Write(CYBSP_UART_HW, em_eeprom_char);
    }
    uart_print("\r\n");

    for (;;)
    {
        /* Toggle the user LED state */
        Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);

        /* Wait for 0.5 seconds */
        Cy_SysLib_Delay(LED_DELAY_MS);
    }
}

/********************************************************************************
* Function Name: handle_error
*********************************************************************************
* Summary: 
* This function processes unrecoverable errors such as any component
* initialization errors etc. In case of such error the system will
* stay in the infinite loop of this function.
*
* Parameters: 
*  uint32_t status: contains the status.
*  char* message: contains the message that is printed to the serial terminal. 
*
* Note: 
*  If error occurs interrupts are disabled.
*
********************************************************************************/
void handle_error(uint32_t status, char *message)
{
    if(CY_EM_EEPROM_SUCCESS != status)
    {
        if(CY_EM_EEPROM_REDUNDANT_COPY_USED != status)
        {
            Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_ON);
            __disable_irq();

            if(NULL != message)
            {
                uart_print(message);
            }

            while(1u);
        }
        else
        {
            uart_print("Main copy is corrupted. Redundant copy in Emulated EEPROM is used \r\n");
        }
    }
}

/********************************************************************************
* Function Name: uart_print
*********************************************************************************
* Summary: 
* This function prints the message to the serial terminal
*
* Parameters: 
*  char* message: message that is printed to the serial terminal. 
*
********************************************************************************/
void uart_print(char *message)
{
    if(NULL != message)
    {
        Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    }
}

/* [] END OF FILE */
