TOP=../../../../../
include $(TOP)Make.rules

TARGET=$(TOP)/Common/corba/LLVS.jar
JYHOME=/usr/share/java
CURRENT=$(PWD)
JC=/usr/j2sdk1.4.1/bin/javac
JAR=/usr/j2sdk1.4.1/bin/jar
JCLASSPATH=/usr/local/IONA/OBE/2.2/lib/OBE.jar:/usr/local/IONA/OBE/2.2/lib/OBENaming.jar:$(TOP)/Common/corba/openhrpstubskel.jar
PACKAGEDIR=jp/go/aist/jrl/vision/
all:  $(TARGET)

$(TARGET):
	$(JC) -d . -classpath $(JCLASSPATH)  ./$(PACKAGEDIR)/HRP2CameraPackage/*.java \
	./$(PACKAGEDIR)/CameraPackage/*.java ./$(PACKAGEDIR)/ModelInfoPackage/*.java \
	./$(PACKAGEDIR)/LowLevelVisionSystemPackage/*.java \
	  ./$(PACKAGEDIR)/*.java;
	$(JAR) cvf $@ jp;

clean:
	rm -f $(TARGET)
