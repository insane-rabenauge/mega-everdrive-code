TARGET=runedx7
SOURCES=\
runedx7.c\

CC=gcc
CFLAGS=-Wall -O2
LDFLAGS=
OBJECTS=$(SOURCES:.c=.o)

.PHONY:	all
all:	$(TARGET)

.PHONY:	run
run:	$(TARGET)
	./$(TARGET) test.bin

.PHONY: clean
clean:	
	@rm -f $(OBJECTS)

.PHONY: backup
backup:
	find . -maxdepth 1 ! -name \*.o -type f -print0 | tar cfz `date +old/v%Y%m%d%H%M.tar.gz` --null -T -

$(TARGET): $(OBJECTS) 
	$(CC) $(OBJECTS) -s -o $@ $(LDFLAGS)

%.o : %.c
	$(CC) $(CFLAGS) -c $<

