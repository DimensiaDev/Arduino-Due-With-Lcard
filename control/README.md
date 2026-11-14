## Как собирать
### pyinstaller
```bash
$ pyinstaller --onefile --windowed --icon "../icon.ico" main.py
```

### nuitka
> **IMPORTANT: Run from x64 Native Tools Command Prompt**
```bash
$ nuitka --onefile --windows-console-mode=disable --windows-icon-from-ico="../icon.ico" --clang --enable-plugins=pyside6 main.py
```
