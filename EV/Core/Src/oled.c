#include "oled.h"
#include "string.h"
#include "font.h"
#include "stdint.h"
#include "stdlib.h"

#define OLED_ADDRESS 0x78
#define OLED_PAGE 8            // OLED页数
#define OLED_ROW 8 * OLED_PAGE // OLED行数
#define OLED_COLUMN 128        // OLED列数
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];
UI_StateTypedef UI_Data;

void OLED_SendCmd(uint8_t cmd){
	uint8_t sendBuffer[2];
	sendBuffer[0] = 0x00;
	sendBuffer[1] = cmd;
  HAL_I2C_Master_Transmit(&hi2c3, OLED_ADDRESS, sendBuffer, sizeof(sendBuffer), 10);
}
void OLED_Init(){
		OLED_SendCmd(0xAE);  // 关显示
    OLED_SendCmd(0x40);  // 显示起始行0
    OLED_SendCmd(0xA1);  // 段反向（0xA0正向，可切换试）
    OLED_SendCmd(0xC8);  // COM反向（若花屏，改为0x08正向）
    OLED_SendCmd(0x81);  // 对比度调节
    OLED_SendCmd(0xCF);  // 对比度值（0xCF=中等亮度）
    OLED_SendCmd(0xA6);  // 正常显示（0xA7反显）
    OLED_SendCmd(0xA8);  // 多路复用率
    OLED_SendCmd(0x3F);  // 64行适配
    OLED_SendCmd(0xD3);  // 显示偏移
    OLED_SendCmd(0x00);  // 0偏移
    OLED_SendCmd(0xD5);  // 时钟分频
    OLED_SendCmd(0x80);  // 标准频率
    OLED_SendCmd(0xD9);  // 预充电周期
    OLED_SendCmd(0xF1);  // SSD1306专用配置
    OLED_SendCmd(0xDA);  // COM引脚配置
    OLED_SendCmd(0x12);  // 64行适配
    OLED_SendCmd(0xDB);  // VCOMH电平
    OLED_SendCmd(0x40);  // 标准电平
    OLED_SendCmd(0x8D);  // 电荷泵
    OLED_SendCmd(0x14);  // 使能
    // --- 关键加在这里：开启水平寻址模式 ---
    OLED_SendCmd(0x20); // 设置寻址模式命令
    OLED_SendCmd(0x00); // 0x00 代表水平寻址：写满一行自动换行，写满一页自动跳下一页
    
    // 设置列和页的范围，确保它知道从哪儿开始，到哪儿结束
    OLED_SendCmd(0x21); OLED_SendCmd(0x00); OLED_SendCmd(0x7F); // 列：0到127
    OLED_SendCmd(0x22); OLED_SendCmd(0x00); OLED_SendCmd(0x07); // 页：0到7
    // ------------------------------------
    OLED_SendCmd(0xAF);  // 开显示
}

//uint8_t GRAM[8][128];


void OLED_NewFrame(){
	memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

//记得测试一下看能不能跑
void OLED_ShowFrame(){
	//uint8_t sendBuffer[129];
  static uint8_t sendBuffer[1025];
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY);
	sendBuffer[0] = 0x40;   //数据标志
  memcpy(&sendBuffer[1], OLED_GRAM, 1024);
  HAL_I2C_Master_Transmit_DMA(&hi2c3, OLED_ADDRESS, sendBuffer, sizeof(sendBuffer));
}

void OLED_SetPixel(uint8_t x, uint8_t y){
	if(x >= 128 || y >= 64) return;
	OLED_GRAM[y/8][x] |= 0x01 << (y % 8);
}

void OLED_Test(){
//	 OLED_NewFrame();
//	 for(uint8_t x = 0; x < 32; x++) {
//        GRAM[0][x] = 0xFF;
//    }
//    OLED_ShowFrame(); 
//	OLED_SendCmd(0xB0);
//	OLED_SendCmd(0x00);
//	OLED_SendCmd(0x10);
  uint8_t sendBuffer[] = {0x40,
		0xe5,0x8f,0xaa,0x00,0x00,0x00,0xfe,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0xfe,0x00,0x00,0x10,0x10,0x18,0x0c,0x06,0x02,0x00,0x00,0x02,0x02,0x04,0x0c,0x18,0x10,
		0xe5,0x9b,0xa0,0x00,0x00,0xff,0x01,0x11,0x11,0x91,0xfd,0x91,0x91,0x11,0x11,0x01,0xff,0x00,0x00,0x3f,0x10,0x16,0x13,0x11,0x10,0x10,0x11,0x13,0x16,0x10,0x3f,0x00,
		0xe4,0xbd,0xa0,0x00,0xc0,0x30,0xf8,0x07,0x21,0x38,0x8e,0x05,0x04,0xe4,0x04,0x84,0xbc,0x0c,0x00,0x00,0x3f,0x00,0x08,0x0e,0x03,0x20,0x20,0x3f,0x00,0x00,0x0f,0x08,
		0xe5,0xa4,0xaa,0x00,0x00,0x08,0x08,0x08,0x08,0x88,0xff,0x7f,0xc8,0x08,0x08,0x08,0x08,0x00,0x30,0x10,0x18,0x0c,0x06,0x01,0x0c,0x18,0x01,0x03,0x06,0x18,0x10,0x30,
		0xe7,0xbe,0x8e,0x00,0x00,0xb4,0xb4,0xb5,0xb7,0xb4,0xfc,0xfc,0xb4,0xb7,0xb5,0xb4,0xb4,0x00,0x22,0x32,0x12,0x12,0x1a,0x0a,0x07,0x07,0x0e,0x1a,0x12,0x12,0x32,0x02,
	};

	HAL_I2C_Master_Transmit(&hi2c3, OLED_ADDRESS, sendBuffer, sizeof(sendBuffer), 50);
}

//画圆
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r){

  int16_t a = 0, b = r, di = 3 - (r << 1);
  while (a <= b)
  {
    OLED_SetPixel(x - b, y - a);
    OLED_SetPixel(x + b, y - a);
    OLED_SetPixel(x - a, y + b);
    OLED_SetPixel(x - b, y - a);
    OLED_SetPixel(x - a, y - b);
    OLED_SetPixel(x + b, y + a);
    OLED_SetPixel(x + a, y - b);
    OLED_SetPixel(x + a, y + b);
    OLED_SetPixel(x - b, y + a);
    a++;
    if (di < 0)
    {
      di += 4 * a + 6;
    }
    else
    {
      di += 10 + 4 * (a - b);
      b--;
    }
    OLED_SetPixel(x + a, y + b);
  }
}

//画图片
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end)
{
  static uint8_t temp;
  if (page >= OLED_PAGE || column >= OLED_COLUMN)
    return;
//  if (color)
//    data = ~data;

  temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
  OLED_GRAM[page][column] &= temp;
  temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
  OLED_GRAM[page][column] |= temp;
  // 使用OLED_SetPixel实现
//   for (uint8_t i = start; i <= end; i++) {
//    OLED_SetPixel(column, page * 8 + i, !((data >> i) & 0x01));
//   }
}

/**
 * @brief 设置显存中的一字节数据
 * @param page 页地址
 * @param column 列地址
 * @param data 数据
 * @param color 颜色
 * @note 此函数将显存中的某一字节设置为data的值
 */
void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data)
{
  if (page >= OLED_PAGE || column >= OLED_COLUMN)
    return;
//  if (color)
//    data = ~data;
  OLED_GRAM[page][column] = data;
}

/**
 * @brief 设置显存中的一字节数据的某几位
 * @param x 横坐标
 * @param y 纵坐标
 * @param data 数据
 * @param len 位数
 * @param color 颜色
 * @note 此函数将显存中从(x,y)开始向下数len位设置为与data相同
 * @note len的范围为1-8
 * @note 此函数与OLED_SetByte_Fine的区别在于此函数的横坐标和纵坐标是以像素为单位的, 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len)
{
  uint8_t page = y / 8;
  uint8_t bit = y % 8;
  if (bit + len > 8)
  {
    OLED_SetByte_Fine(page, x, data << bit, bit, 7);
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, len + bit - 1 - 8);
  }
  else
  {
    OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1);
  }
  // 使用OLED_SetPixel实现
  // for (uint8_t i = 0; i < len; i++) {
  //   OLED_SetPixel(x, y + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief 设置显存中一字节长度的数据
 * @param x 横坐标
 * @param y 纵坐标
 * @param data 数据
 * @param color 颜色
 * @note 此函数将显存中从(x,y)开始向下数8位设置为与data相同
 * @note 此函数与OLED_SetByte的区别在于此函数的横坐标和纵坐标是以像素为单位的, 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data)
{
  uint8_t page = y / 8;
  uint8_t bit = y % 8;
  OLED_SetByte_Fine(page, x, data << bit, bit, 7);
  if (bit)
  {
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1);
  }
}

/**
 * @brief 设置一块显存区域
 * @param x 起始横坐标
 * @param y 起始纵坐标
 * @param data 数据的起始地址
 * @param w 宽度
 * @param h 高度
 * @param color 颜色
 * @note 此函数将显存中从(x,y)开始的w*h个像素设置为data中的数据
 * @note data的数据应该采用列行式排列
 */
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h)
{
  uint8_t fullRow = h / 8; // 完整的行数
  uint8_t partBit = h % 8; // 不完整的字节中的有效位数
  for (uint8_t i = 0; i < w; i++)
  {
    for (uint8_t j = 0; j < fullRow; j++)
    {
      OLED_SetBits(x + i, y + j * 8, data[j * w + i]);
    }
  }
  if (partBit)
  {
    uint16_t fullNum = w * fullRow; // 完整的字节数
    for (uint8_t i = 0; i < w; i++)
    {
      OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit);
    }
  }
}

void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img)
{
  OLED_SetBlock(x, y, img->data, img->w, img->h);
}

// ================================ 文字绘制 ================================

/**
 * @brief 绘制一个ASCII字符
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param ch 字符
 * @param font 字体
 * @param color 颜色
 */
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font)
{
  OLED_SetBlock(x, y, font->chars + (ch - ' ') * (((font->h + 7) / 8) * font->w), font->w, font->h);
}

/**
 * @brief 绘制一个ASCII字符串
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param str 字符串
 * @param font 字体8
 * @param color 颜色
 */
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font)
{
  uint8_t x0 = x;
  while (*str)
  {
    OLED_PrintASCIIChar(x0, y, *str, font);
    x0 += font->w;
    str++;
  }
}

/**
 * @brief 获取UTF-8编码的字符长度
 */
uint8_t _OLED_GetUTF8Len(char *string)
{
  if ((string[0] & 0x80) == 0x00)
  {
    return 1;
  }
  else if ((string[0] & 0xE0) == 0xC0)
  {
    return 2;
  }
  else if ((string[0] & 0xF0) == 0xE0)
  {
    return 3;
  }
  else if ((string[0] & 0xF8) == 0xF0)
  {
    return 4;
  }
  return 0;
}

/**
 * @brief 绘制字符串
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param str 字符串
 * @param font 字体
 * @param color 颜色
 *
 * @note 为保证字符串中的中文会被自动识别并绘制, 需:
 * 1. 编译器字符集设置为UTF-8
 * 2. 使用波特律动LED取模工具生成字模(https://led.baud-dance.com)
 */
/**
 * @brief 绘制字符串
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param str 字符串
 * @param font 字体
 * @param color 颜色
 *
 * @note 为保证字符串中的中文会被自动识别并绘制, 需:
 * 1. 编译器字符集设置为UTF-8
 * 2. 使用波特律动LED取模工具生成字模(https://led.baud-dance.com)
 */
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font)
{
  uint16_t i = 0;                                       // 字符串索引
  uint8_t oneLen = (((font->h + 7) / 8) * font->w) + 4; // 一个字模占多少字节
  uint8_t found;                                        // 是否找到字模
  uint8_t utf8Len;                                      // UTF-8编码长度
  uint8_t *head;                                        // 字模头指针
  while (str[i])
  {
    found = 0;
    utf8Len = _OLED_GetUTF8Len(str + i);
    if (utf8Len == 0)
      break; // 有问题的UTF-8编码

    // 寻找字符  TODO 优化查找算法, 二分查找或者hash
    for (uint8_t j = 0; j < font->len; j++)
    {
      head = (uint8_t *)(font->chars) + (j * oneLen);
      if (memcmp(str + i, head, utf8Len) == 0)
      {
        OLED_SetBlock(x, y, head + 4, font->w, font->h);
        // 移动光标
        x += font->w;
        i += utf8Len;
        found = 1;
        break;
      }
    }

    // 若未找到字模,且为ASCII字符, 则缺省显示ASCII字符
    if (found == 0)
    {
      if (utf8Len == 1)
      {
        OLED_PrintASCIIChar(x, y, str[i], font->ascii);
        // 移动光标
        x += font->ascii->w;
        i += utf8Len;
      }
      else
      {
        OLED_PrintASCIIChar(x, y, ' ', font->ascii);
        x += font->ascii->w;
        i += utf8Len;
      }
    }
  }
}

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  static uint8_t temp = 0;
  if (x1 == x2) {
    if (y1 > y2) {
      temp = y1;
      y1 = y2;
      y2 = temp;
    }
    for (uint8_t y = y1; y <= y2; y++) {
      OLED_SetPixel(x1, y);
    }
  } else if (y1 == y2) {
    if (x1 > x2) {
      temp = x1;
      x1 = x2;
      x2 = temp;
    }
    for (uint8_t x = x1; x <= x2; x++) {
      OLED_SetPixel(x, y1);
    }
  } else {
    // Bresenham直线算法
    int16_t dx = x2 - x1;
    int16_t dy = y2 - y1;
    int16_t ux = ((dx > 0) << 1) - 1;
    int16_t uy = ((dy > 0) << 1) - 1;
    int16_t x = x1, y = y1, eps = 0;
    dx = abs(dx);
    dy = abs(dy);
    if (dx > dy) {
      for (x = x1; x != x2; x += ux) {
        OLED_SetPixel(x, y);
        eps += dy;
        if ((eps << 1) >= dx) {
          y += uy;
          eps -= dx;
        }
      }
    } else {
      for (y = y1; y != y2; y += uy) {
        OLED_SetPixel(x, y);
        eps += dx;
        if ((eps << 1) >= dy) {
          x += ux;
          eps -= dy;
        }
      }
    }
  }
}

/**
 * @brief 绘制一个矩形
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param w 矩形宽度
 * @param h 矩形高度
 * @param color 颜色
 */
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  OLED_DrawLine(x, y, x + w, y);
  OLED_DrawLine(x, y + h, x + w, y + h);
  OLED_DrawLine(x, y, x, y + h);
  OLED_DrawLine(x + w, y, x + w, y + h);
}

/**
 * @brief 绘制一个填充矩形
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param w 矩形宽度
 * @param h 矩形高度
 * @param color 颜色
 */
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  for (uint8_t i = 0; i < h; i++) {
    OLED_DrawLine(x, y + i, x + w, y + i);
  }
}