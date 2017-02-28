VPATH = ../
CFLAGS = -I ../

so = add.so sub.so multiply.so divide.so power.so

all: $(so)

# <targets ...>: <target-pattern>: <prereq-patterns ...>
# 	<commands>
# 	...
# 目标模式或是依赖模式中都应该有%这个字符
# "$<"所有的依赖目标集, "$@"表示目标集

$(so): %.so: %.o
	cc -shared $< -o $@


so_objects = add.o sub.o multiply.o divide.o power.o

all: $(so_objects)

$(so_objects): %.o: %.c calType.h
	cc -fPIC -c $(CFLAGS) $< -o $@


.PHONY: clean
clean:
	rm -rf $(so_objects) $(so)