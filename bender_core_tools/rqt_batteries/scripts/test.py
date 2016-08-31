import subprocess

cmd = "upower -i $(upower -e | grep 'BAT') | grep -E 'state|to\ full|percentage'"
ps = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
output = ps.communicate()[0]

#print output

conectado=1
if 'discharging' in output:
	conectado=0


print conectado

posicion=output.index('%')

porcentaje=output[posicion-3:posicion]

if porcentaje[0]==' ':
	porcentaje=porcentaje[1:3]

print porcentaje


