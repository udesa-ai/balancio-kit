# Balacio Kit

## Infomación

Proyecto educativo de un robot de autobalanceo de ultra bajo costo, capaz de correr una red neuronal para mantener el equibrio y de ser controlado remotamente de manera inalámbrica.

Desarrollado con fines didácticos para enseñar conceptos de RL, ML, AI y control.


![Balancio v0.4](resources/Balanciov0_4.jpg?raw=true )

## Calibración

Estas instrucciones asumen conocimiento de el uso de la IDE arduino
### Compensación IMU
1. Abrir `Balancio-kit/Code/Mcu/Src/imu_calibration/imu_calibration.ino` con el IDE Arduino
2. poner la IMU paralela al piso y mantenerla firme

3. subir el programa a la placa y usar el monitor serial para obtener las compensaciones de la IMU

4. modificar las copensaciones en el archivo `PID.ino`
### Calibrar angulo de equilibrio
1. Abrir `Balancio-kit/Code/Mcu/Src/pid/pid.ino`

2. sostener el robot en la posición de equilibrio

3. subir el programa a la placa y usar el monitor serial para obtener las compensaciones de angulo

4. modificar en angulo de quilibrio en el archivo `PID.ino`

### Calibracion constantes PID
1. Sacar el jumper de 12v en el driver 

2. Elegir parametros por algun método como Zigler-Nichols

3. Probar las constantes, si se sacó el jumper se puede probar incluso con el cable conectado **cuidado al hacer esto!**
---

## TODO:

- [x] initial commit
- [ ] Desarrollar aplicación bluetooth
- [ ] Crear agente RL
- [ ] Diseño mecánico
- [ ] Publicar STEP del diseño mecánico
- [ ] Crear diagrama electrónico

## Bugs conocidos:

- Wheel spins on sturtup

## Contribuciones
Las *pull requests* son bienvenidas, para cambios mayores, por favor abrir un *issue* para discutir los cambios deseados

## Licencia
[MIT](https://choosealicense.com/licenses/mit/)