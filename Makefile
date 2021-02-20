#Fichier Makefile
#Auteur Sylvain Jalbert
#Date 15 fevrier 2021

CC = sudo gcc -Wall -lwiringPi -lm
RM = rm -f
srcdir = src/
bindir = bin/
docdir = doc/
savedir = save/
SRC = $(wildcard $(srcdir)*.c)
HEAD = $(wildcard *.h)
OBJ = $(subst $(srcdir), $(bindir), $(SRC:.c=.o))
PROG = robotdog

all : $(PROG)

$(PROG) : $(OBJ) #Compilation du programme
	$(CC) $^ -o $@

./bin/%.o : ./src/%.c #Compilation des objets
	$(CC) -c $^ -o $@

.PHONY : clean #Regle de contournement de fichier appele clean

.PHONY : doc #Regle de contournement de fichier appele doc

clean : #Suppression des objets et le la documentation
	$(RM) $(OBJ)
	$(shell rm -rf doc/html doc/latex)

doc : #Generation de la documentation Doxygen
	$(shell doxygen Doxyfile)

dir :
	$(shell mkdir src bin doc)
