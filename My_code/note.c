/*
    USER CODE BEGIN 1 ——“初始化前的准备区”
    适合写一些全局变量的定义和软件模块初始化（不碰硬件）等
    USER CODE BEGIN 2 ——“真正开始跑系统”
    适合写启动外设（Start），打开中断，DMA等
    在用cubemx添加功能再生成代码前先保存
*/

/*
 使用sprintf函数时，默认情况下，编译器会链接newlib-nano库，这个库为了节省空间，默认不包含浮点数支持。
 因此，如果你在代码中使用了sprintf来格式化浮点数（如%f），你需要确保链接器将包含浮点数支持的版本的sprintf函数链接到你的项目中。
    你可以通过在CMakeLists.txt文件最下方中添加以下链接选项来实现这一点：
# Ensure printf floating-point support is linked (newlib-nano)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-Wl,-u,_printf_float")

*/



/*   增加.c文件和.h文件到工程中 (CMakeLists.txt)

    # Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Unitree/crc_ccitt.c
    Unitree/gom_protocol.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Unitree
)

*/

/*  uart_dma功能配置说明：

    接受配置
    (需要在usart.h中添加变量声明)
    extern DMA_HandleTypeDef hdma_usart1_rx;
    extern DMA_HandleTypeDef hdma_usart1_tx;
    extern uint8_t pData[10]; 
    (需要在usart.c中添加变量定义)
    uint8_t pData[10];

    （1）空闲中断配置
    (需要在main.c的main函数中添加初始化代码)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)pData, sizeof(pData));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    (需要在usart.c中添加回调函数)
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        if(huart->Instance == USART1) //check if the interrupt comes from USART1
        {
            HAL_UART_Transmit_DMA(&huart1, pData, Size); //echo the received data
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pData, sizeof(pData));
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        }
    }

    （2)正常中断配置(若是不使用DMA，则将下文中_DMA_替换为_IT即可)
    (需要在main.c的main函数中添加初始化代码)
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)pData, sizeof(pData));
    (需要在usart.c中添加回调函数)
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
        if(huart->Instance == USART1) //check if the interrupt comes from USART1
        {
            HAL_UART_Transmit_DMA(&huart1, pData, 5); //echo the received data
            HAL_UART_Receive_DMA(&huart1, pData, 5); //re-enable the interrupt
        }
    }

*/

/*
    ozone Debug看波形时候，因为初始单位较小会有较大振幅，属正常现象
*/





