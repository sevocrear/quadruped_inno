f = open("save.txt","w+") 
import time
# Then you want to write the stuff to it. 
output = str([1]*5000)
st = time.time()
f.write(output)
print(time.time() - st) 
# Then close the file 
f.close() 