# UART
H750
# demo1
实现串口发送 
# demo2
移植stdio的printf 
# demo3
移植BDMA+IDLE串口接收 
#### **注意**
之前一直调不通是因为 BDMA只能访问SRAM4、APB4、APH4和备份RAM P95 
SRAM4的地址是0x3800 0000	P104 
需要ALIGN_32BYTES ( unsigned char   rx_buffer[200]) __attribute__((section(".ARM.__at_0x38000000"))); 
把变量地址设置过去 