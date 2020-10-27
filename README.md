# *Установка*

Весь код взят из https://github.com/relativty/Relativty. Там есть инструкция, но, продублирую здесь.

Сначала установить Python 3.8.4. Затем установи `PyTorch`:

`python -m pip install torch===1.6.0 torchvision===0.7.0 -f https://download.pytorch.org/whl/torch_stable.html`

Далее установи `CUDA Toolkit 11.0.`

Установка самого драйвера в стим:

- Locate your `vrpathreg.exe` program, usually located at `C:/Steam/steamapps/common/SteamVR/bin/win64/vrpathreg.exe`
- Then open the Windows Command Prompt and run the following commands:
`cd C:/Steam/steamapps/common/SteamVR/bin/win64
vrpathreg.exe`

And then assuming your `Relativty_Driver/Relativty` driver folder is located at:
`C:/code/Relativty_Driver/Relativty`
- run `vrpathreg adddriver C:/code/Relativty_Driver/Relativty`

Relativty Driver is now installed. You can uninstall it any time by running:  
- `vrpathreg removedriver C:/code/Relativty_Driver/Relativty`

# *Запуск*

На момент записи этого ридми необходимо просто запустить SteamVr, затем вручную запустить скрипт Client.py в Relativty/resources/PYTHONPATH/.
У скрипта могут быть проблемы с выбором web камеры(Нужно бы сделать какой-то умный выбор видеокамеры). Можно узать ip webcam приложение для телефона
вместо камеры
