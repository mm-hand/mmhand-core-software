import pathlib, codecs 
p=pathlib.Path(r'F:/MMHAND/ESP32-S3/System/src/tasks/EncodersTask.cpp') 
d=p.read_bytes() 
print(d[:4], d[:4].hex(), d.startswith(codecs.BOM_UTF8)) 
