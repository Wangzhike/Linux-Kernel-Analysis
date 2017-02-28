objects = main.o readConfig.o messageTable.o 

.PHONY: all
all: cal so

cal: $(objects)
	cc -o cal $(objects) -ldl -lpthread

main.o: readConfig.h messageTable.h calType.h

messageTable.o: messageTable.h calType.h

readConfig.o: readConfig.h messageTable.h calType.h

.PHONY: so
so:
	cd so && $(MAKE) -w


.PHONY: clean
clean:
	rm -rf cal $(objects); cd so && $(MAKE) clean -w