global t
global y

with open('dat.dat', 'rb') as f:
	y = f.read()
with open('tmpt', "rb") as f:
	t = f.read()

y_l = y.split("\n".encode('utf8'))
t_l = t.split("\n".encode('utf8'))

print("leny:", len(y_l), "lent:", len(t_l))

with open("costura", "wb") as f:
	for i in range(0, len(y_l)):
		f.write("{} {}\n".format(t_l[i].decode('utf8'), y_l[i].decode('utf8')).encode('utf8'))
