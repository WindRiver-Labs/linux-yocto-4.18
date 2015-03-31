import sys

while len(sys.argv) > 1:
	f = open(sys.argv.pop(1), "rb")
	f.seek(0, 2)
	sz = f.tell()
	f.seek(0,0)
	pad = ((sz + 3) & ~3) - sz
	sys.stdout.write(f.read() + '\0'*pad)
	f.close()
