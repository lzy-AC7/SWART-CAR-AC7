#include "uart.h"

void uart_rec_init()
{
    fifo_init(&uart_rec_data_fifo, FIFO_DATA_8BIT, uart_rec_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_REC_INDEX, UART_REC_BAUDRATE, UART_REC_TX_PIN, UART_REC_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    uart_rx_interrupt(UART_REC_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_REC_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

}

void uart_map_init()
{
    fifo_init(&uart_map_data_fifo, FIFO_DATA_8BIT, uart_map_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_MAP_INDEX, UART_MAP_BAUDRATE, UART_MAP_TX_PIN, UART_MAP_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    uart_rx_interrupt(UART_MAP_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_MAP_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

}

void uart_rec_rx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_REC_INDEX, &get_rec_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_rec_data_fifo, &get_rec_data, 1);                           // 将数据写入 fifo 中
}

void uart_map_rx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_MAP_INDEX, &get_map_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_map_data_fifo, &get_map_data, 1);                           // 将数据写入 fifo 中
}

bool wait_for_response(fifo_struct *fifo,uint8* buffer, uint32 *count,uint32_t timeout_ms) 
{
    uint32_t start_time = sys_time; // 假设你有获取系统时间的函数
    *count = 0;

    while ((sys_time - start_time) < timeout_ms) 
    {
        uint32_t current_used = fifo_used(fifo);
        if (current_used > 0) 
        {
            // 读取数据
            fifo_read_buffer(fifo, (uint8_t*)&buffer[*count], &current_used, FIFO_READ_AND_CLEAN);
            *count += current_used;
            
            // 2. 检查是否接收到换行符（OpenMV 发送以 \n 结尾）
            if (buffer[*count - 1] == '\n') 
            {
                buffer[*count - 1] = '\0'; // 替换为字符串结束符
                return 1; // 接收成功
            }
        }
        system_delay_ms(5);
    }
    return 0;
}

void process_pos_data(void)
{
    if(maping) return;
    uint8_t header;
    uint32_t len;
    // 【已修改】当前 1 帧定点数包的总长度仅 6 字节（1帧头 + 4数据 + 1校验）
    while (fifo_used(&uart_map_data_fifo) >= 6) 
    {
        
        // 1. 尝试读取 1 字节看是不是帧头
        len = 1;
        fifo_read_buffer(&uart_map_data_fifo, &header, &len, FIFO_READ_ONLY); 
        
        if (header != 0xAC) {
            // 如果不是帧头，扔掉这 1 字节，继续找下一个
            len = 1;
            fifo_read_buffer(&uart_map_data_fifo, &header, &len, FIFO_READ_AND_CLEAN);
            continue; 
        }

        // 2. 确实是帧头，读取整帧 (6 字节) 进行判断
        // 先只读不删，防止校验失败导致数据错位丢包
        uint8_t full_frame[6];
        len = 6;
        fifo_read_buffer(&uart_map_data_fifo, full_frame, &len, FIFO_READ_ONLY);

        // 3. 计算校验和 (计算中间 4 个字节的数据部分：full_frame[1] 到 [4])
        uint8_t sum = 0;
        for(int i = 1; i <= 4; i++) {
            sum += full_frame[i];
        }

        // 【已修改】校验和现在位于 full_frame[5]
        if (sum == full_frame[5]) 
        {
            // 4. 校验成功！强转读取 uint16_t 数据，并除以 1000.0f 还原成 float
            uint16_t raw_x = *((uint16_t*)&full_frame[1]);
            uint16_t raw_y = *((uint16_t*)&full_frame[3]);
            
            art_x =  1.0*(200+200*raw_x / 1000.0f);
            art_y = -1.0*(200+200*raw_y / 1000.0f);

            // 从 FIFO 中彻底删除这已处理的 6 字节
            len = 6;
            fifo_read_buffer(&uart_map_data_fifo, full_frame, &len, FIFO_READ_AND_CLEAN);
            ready = 1;
            
            // printf("%f,%f\n", art_x, art_y); // 打印调试
        } else {
            // 校验失败，说明这个 0xAC 只是普通数据或者是错位的包
            // 仅弹出这一个错误的帧头，继续找下一个
            len = 1;
            fifo_read_buffer(&uart_map_data_fifo, &header, &len, FIFO_READ_AND_CLEAN);
        }
    }
}

void process_map_data(void)
{
    maping = 1;
    bool res_success = 0;
    do
    {
        uart_write_byte(UART_MAP_INDEX, '1');
        fifo_clear(&uart_map_data_fifo);
        res_success = wait_for_response(&uart_map_data_fifo,fifo_map_get_data, &fifo_map_data_count,100);
    } while (!res_success || (fifo_map_get_data[0] != '1'));
    if(wait_for_response(&uart_map_data_fifo,fifo_map_get_data, &fifo_map_data_count,3500))
    {
        for(int i = 0;i < ROW;i++)
            for(int j = 0;j < COL;j++)
                map_txt[i][j] = fifo_map_get_data[i*COL+j];
        for(int i = 0;i < ROW;i++)
        {
            for(int j = 0;j < COL;j++)
                printf("%3c",map_txt[i][j]);
            putchar('\n');
        }
    }
    res_success = 0;
    do
    {
        uart_write_byte(UART_MAP_INDEX, '0');
        fifo_clear(&uart_map_data_fifo);
        res_success = wait_for_response(&uart_map_data_fifo,fifo_map_get_data, &fifo_map_data_count,100);
    } while (!res_success || (fifo_map_get_data[0] != '0'));
    maping = 0;
}

bool process_rec_data(bool mod)
{
    maping = 1;
    bool res_success = 0;
    do
    {
        uart_write_byte(UART_MAP_INDEX, '2');
        fifo_clear(&uart_map_data_fifo);
        res_success = wait_for_response(&uart_map_data_fifo,fifo_map_get_data, &fifo_map_data_count,100);
    } while (!res_success || (fifo_map_get_data[0] != '2'));

    bool rec_success = 0;
    do
    {
        uart_write_byte(UART_REC_INDEX, '0'+ mod);
        fifo_clear(&uart_rec_data_fifo);
        rec_success = wait_for_response(&uart_rec_data_fifo,fifo_rec_get_data, &fifo_rec_data_count,100);
    } while (!rec_success || (fifo_rec_get_data[0] != '0'+ mod));
    bool b = wait_for_response(&uart_rec_data_fifo,fifo_rec_get_data, &fifo_rec_data_count,3500);

    res_success = 0;
    do
    {
        uart_write_byte(UART_MAP_INDEX, '0');
        fifo_clear(&uart_map_data_fifo);
        res_success = wait_for_response(&uart_map_data_fifo,fifo_map_get_data, &fifo_map_data_count,100);
    } while (!res_success || (fifo_map_get_data[0] != '0'));
    maping = 0;
    return b;
}