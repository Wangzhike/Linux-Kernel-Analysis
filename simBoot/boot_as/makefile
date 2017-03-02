all: boot.img  
  
boot.o: boot.s
	as -o $@ $<  
  
boot: boot.o  
	ld --oformat binary -N -Ttext 0x7c00 -o $@ $<  
  
boot.img: boot  
	dd if=boot of=boot.img bs=512 count=1  
  
clean:  
	rm ./boot ./boot.img ./boot.o
