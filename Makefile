#Fichier Makefile
#Auteur Sylvain Jalbert
#Date 15 fevrier 2021

DEBUG	= -O3
CC	= gcc
INCLUDE	= -I/usr/local/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe
LDFLAGS	= -L/usr/local/lib
LDLIBS	= -lwiringPi -lwiringPiDev -lpthread -lm -lwiringPiPca9685
RM = rm -f
srcdir = src/
bindir = bin/
docdir = doc/
SRC = $(wildcard $(srcdir)*.c)
HEAD = $(wildcard *.h)
OBJ = $(subst $(srcdir), $(bindir), $(SRC:.c=.o))
PROG = robotdog

all : $(PROG)

$(PROG) : $(OBJ) #Compilation du programme
	$(CC) -o $^ $@ $(LDFLAGS) $(LDLIBS)

./bin/%.o : ./src/%.c #Compilation des objets
	$(CC) -c $(CFLAGS) $^ -o $@

.PHONY : clean #Regle de contournement de fichier appele clean

.PHONY : doc #Regle de contournement de fichier appele doc

clean : #Suppression des objets et le la documentation
	$(RM) $(OBJ)
	$(shell rm -rf doc/html doc/latex)

doc : #Generation de la documentation Doxygen
	$(shell doxygen Doxyfile)

dir :
	$(shell mkdir src bin doc)
