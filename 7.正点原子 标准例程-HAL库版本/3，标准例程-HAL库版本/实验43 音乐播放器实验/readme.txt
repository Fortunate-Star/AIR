实验器材:
	战舰STM32F103开发板V3版本
	
实验目的:
	学习VS1053的使用,实现一个简单的MP3.
	
硬件资源:
	1,DS0(连接在PB5)
	2,串口1(波特率:115200,PA9/PA10连接在板载USB转串口芯片CH340上面)
	3,ALIENTEK 2.8/3.5/4.3/7寸TFTLCD模块(通过FSMC驱动,FSMC_NE4接LCD片选/A10接RS)
	4,按键KEY0(PE4)/KEY1(PE3)/KEY2(PE2)/KEY_UP(PA0,也称之为WK_UP)
	5,SD卡,通过SDIO(SDIO_D0~D4(PC8~PC11),SDIO_SCK(PC12),SDIO_CMD(PD2))连接 
	6,W25Q128(SPI FLASH芯片,连接在SPI2上) 
	7,VS1053与STM32芯片连接关系为: 
	  VS1053解码芯片----------  STM32芯片
		 VS_SCK    -----------   PA5
		 VS_MISO   ------------  PA6
		 VS_MOSI   ------------  PA7
		 VS_RST    ------------  PE6
		 VS_XDCS   -----------   PF6
		 VS_XCS    -----------   PF7
		 VS_DREQ   ------------  PC13
		
实验现象:
	本实验开机先检测字库是否存在，如果检测无问题，则对VS1053进行RAM测试和正弦测试，测试完后开始循环播
	放SD卡MUSIC文件夹里面的歌曲（必须在SD卡根目录建立一个MUSIC文件夹，并存放歌曲在里面），在TFTLCD上
	显示歌曲名字、播放时间、歌曲总时间、歌曲总数目、当前歌曲的编号等信息。KEY0用于选择下一曲，KEY2用于
	选择上一曲，KEY_UP和KEY1用来调节音量。本实验用DS0指示程序运行状况。 
	本例程可以通过USMART来调用VS_Set_Vol/VS_Set_Bass/VS_Set_Effect等函数,实现对VS1053的配置.

注意事项:
	1,4.3寸和7寸屏需要比较大电流,USB供电可能不足,请用外部电源适配器(推荐外接12V 1A电源).
	2,本例程在LCD_Init函数里面(在ILI93xx.c),用到了printf,如果不初始化串口1,将导致液晶无法显示!! 
	3,该实验必须自备SD卡一个,并在SD卡根目录新建MUSIC文件夹，存放一些音乐文件(mp3/wav/wma/flac/midi/
	  ogg)到该文件夹下面。
	4,正弦测试需要插耳机才能听到.板子自带的喇叭不会播放正弦测试的声音.
	
			正点原子@ALIENTEK 
			广州市星翼电子科技有限公司
			电话：020-38271790
			传真：020-36773971
			购买：http://eboard.taobao.com
			公司网站：www.alientek.com
			技术论坛：www.openedv.com
