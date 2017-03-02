;
; Note: this example is written in Intel Assembly syntax
;
[BITS 16]
[ORG  0x7c00]

boot:
	;首先读入光标位置
	mov ah, 0x03		; AH=0x03，读光标位置
	xor bh, bh			; BH=0x00，第0页
	int 0x10

	;显示字符串msg
	mov ah, 0x13		; 显示字符串
    mov al, 0x01		; 光标跟随移动
    mov bh, 0x00		; BH=0x00, 第0页
    mov bl, 0x07		; BH=0x07, 属性7(normal)
	mov cx,	23			; 要显示的字符串长度
	mov bp, msg			; 字符串地址

    int 0x10
    jmp $

msg:
	db "Hello, MBR is runing!"
	;换行+回车
	db 13
	db 10

times 510-($-$$) db 0
;魔术字节，BIOS看到这2个字节，认为这个设备是一个可引导设备
db 0x55
db 0xaa
