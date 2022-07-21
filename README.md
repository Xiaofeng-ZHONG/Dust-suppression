Dust suppression scheme for tailings reservoir

**This program is only for study and communication. Please do not reprint it without permission.**

# Before you run the  Dust suppression code
you need run anothor code  first to store MAC address to SIP FLASH (W25Q32)  [Go to project](https://github.com/Xiaofeng-ZHONG/Store_MAC_address_to_W25Q32.git)


# 1 Main Modules
## 1.1 STM32F401CCU6
  dominant frequency-84MHz, SRAM-64K, FLASH-256K, packing-QFPN48, SIP FLASH-W25Q32 
  ![ad893ab3d014d3fc1fc95df5f497778b](https://user-images.githubusercontent.com/108401612/179658146-8f88fca5-268f-4248-8502-dcb1e77ae88f.jpeg)
## 1.2 EC-01 Kit
  ![an3](https://user-images.githubusercontent.com/108401612/179657241-ed162006-9c77-41fc-818f-84a569f4c783.jpg)
## 1.3 E18-MS1-PCB
  ![E18](https://user-images.githubusercontent.com/108401612/179658030-096f7f5b-2a3c-4448-a306-b3b46e85cae9.png)
# 2 Development tools
  ## 2.1 STM32CubeMX.exe
  ![image](https://user-images.githubusercontent.com/108401612/179658711-da826e9d-8fb3-44d0-8a7f-65acc4a8e1f0.png)
  [Download STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
  ## 2.2 Keil uVision5
  ![image](https://user-images.githubusercontent.com/108401612/179658677-0d8b5a09-6de5-4f1c-8fbd-50b6d524962f.png)
  [Download Keil uVision5](https://www2.keil.com/mdk5)
  ## 2.3 Serial port communication tool
  You need to search the Internet for such software
# 3 Progect File
  + /Dust_suppression.ioc                          (use STM32CubeMX to open)
  + /MDK-ARM/Dust_suppression.uvprojx              (use Keil uVision5 to open)
# 4 Attention
  ## 4.1
  ![image](https://user-images.githubusercontent.com/108401612/179661692-c8905532-a0f0-4200-82b7-d8ba090db1d3.png)
  ## 4.2
  ![image](https://user-images.githubusercontent.com/108401612/179661829-7f6a1c7f-8efc-479c-af9d-e53bcd70ec8a.png)
  ## 4.3
  ![image](https://user-images.githubusercontent.com/108401612/179661506-bef00475-fd63-467b-8220-ae84eb4bd3ee.png)
# 5 Modify here
![image](https://user-images.githubusercontent.com/108401612/180137603-25226812-be3c-4e7c-8734-e9d06de9af91.png)


