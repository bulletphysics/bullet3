
CC = gcc
COPTS = -O2 -Wall

#CC = gcc31
#COPTS = -O2 -Wall -ggdb -lm

#COPTS = -O3 -g -fno-inline-functions -ffast-math -pg -Wall
#COPTS = -ggdb -Wall

CFLAGS = $(COPTS) $(INCS) -lm

EXEC = testblend

OBJ_SRCS = testblend.c readblend.c
OBJ_OBJS = $(OBJ_SRCS:.c=.o)

%.o: %.c
	${CC} ${CFLAGS} -c $< -o $@

all: $(EXEC)

clean:
	rm -f $(EXEC) *.o gmon.out

#dist:
#	tar -hzvcf flynn-`cat VERSION`.tar.gz `cat MANIFEST`
#	ls -la flynn-`cat VERSION`.tar.gz

#testdist:
#	tar -zvcf flynn-testdir.tar.gz test
#	ls -la flynn-testdir.tar.gz

$(EXEC): $(OBJ_OBJS)
	$(CC) $(CFLAGS) $^ -o $@

