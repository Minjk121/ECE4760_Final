import binascii
from tkinter import filedialog

file_path = filedialog.askopenfilename()

x = ""
with open(file_path, 'rb') as f:
    for chunk in iter(lambda: f.read(32), b''):
        x += str(binascii.hexlify(chunk)).replace("b","").replace("'","")
b = bin(int(x, 16)).replace('b','')
g = [b[i:i+2] for i in range(0, len(b), 2)]
dna = ""
for i in g:
    if i == "00":
        dna += "A"
    elif i == "01":
        dna += "T"
    elif i == "10":
        dna += "G"
    elif i == "11":
        dna += "C"
with open ("wavBin.txt","w") as f:
    print(str(list(b))[1:-1].replace("'",""), file=f)
#print(x) #hexdump
#print(type(b)) #converted to binary
#print(dna) #converted to "DNA"