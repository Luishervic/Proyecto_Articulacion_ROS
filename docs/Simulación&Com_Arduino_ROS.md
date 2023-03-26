# Simulación en ROS.
---
- [Simulación en ROS.](#simulación-en-ros)
  - [1. Exportar simulación al formato URDF.](#1-exportar-simulación-al-formato-urdf)
  - [2. Comunicación ROS - Arduino.](#2-comunicación-ros---arduino)
    - [2.1 Instalación de la libreria rosserial.](#21-instalación-de-la-libreria-rosserial)
    - [2.2 Instalar paquete rosserial.](#22-instalar-paquete-rosserial)
    - [2.3 Encontrar el puerto USB.](#23-encontrar-el-puerto-usb)
    - [2.4 Agregar permisos.](#24-agregar-permisos)
    - [2.5 Prueba de comunicación.](#25-prueba-de-comunicación)
  - [3. Videotutorial de la sección.](#3-videotutorial-de-la-sección)

## 1. Exportar simulación al formato URDF.
El archivo URDF es esencial para trabajar con ROS y RViz, ya que describe la geometría, cinemática y dinámica de un robot de forma detallada y estructurada. Con este archivo, se puede visualizar y simular el robot en RViz, y también puede ser utilizado por otros paquetes de ROS para controlar y planificar el movimiento del robot.

<div align="center">
    <img src="/imgs/splash.png">
</div>

Antes de exportar un archivo URDF desde Fusion, es necesario contar con las herramientas adecuadas para realizar esta tarea. La herramienta principal que se requiere es el URDF Exporter, una extensión de Fusion 360 que permite exportar modelos de Fusion en formato URDF para ser utilizados en ROS. Para instalarla seguimos los siguientes pasos:

1. Abre el navegador web y visita la página del repositorio oficial [fusion2urdf](https://github.com/syuntoku14/fusion2urdf).

2. Haz clic en el botón "Code" en la parte superior derecha de la página.

3. Selecciona "Download ZIP" para descargar un archivo ZIP con la herramienta.

4. Descomprime el archivo ZIP descargado.

5. Abre PowerShell en tu computadora.
   
6. Copia la ruta donde descomprimiste el archivo ZIP descargado.
   
7. En PowerShell, escribe el siguiente comando en la línea de comandos, reemplazando "ruta a fusion2urdf" con la ruta donde descomprimiste el archivo ZIP:

```
cd <ruta a fusion2urdf>
```
```
Copy-Item ".\URDF_Exporter\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

8. Abre Fusion 360 y el archivo del modelo que deseas exportar.
9.  Selecciona "Add-Ins" en la barra de herramientas de la parte superior de la ventana de Fusion 360.
10. Selecciona el archivo URDF_Exporter descargado previamente y haz clic en "Run".

---
## 2. Comunicación ROS - Arduino.
En los siguientes pasos se asume que el lector ya tiene instalado el IDE de Arduino.
### 2.1 Instalación de la libreria rosserial. 
Para instalar la herramienta, simplemente sigue estos pasos: 
* Abre el IDE de Arduino en tu computadora Windows.

* En la barra de menú superior, selecciona "Herramientas" y luego "Administrador de librerías".

* En la ventana del Administrador de librerías, busca "rosserial" en el cuadro de búsqueda en la esquina superior derecha.

* Aparecerán varias opciones de rosserial. Selecciona la versión que deseas descargar e instalar. Asegúrate de seleccionar la versión que sea compatible con la versión de tu sistema operativo y el IDE de Arduino que estés utilizando.

* Haz clic en el botón "Instalar" en la parte inferior derecha de la ventana.

* Espera a que se complete la instalación de la librería.

Una vez que la instalación haya finalizado, puedes cerrar la ventana del Administrador de librerías.

---

### 2.2 Instalar paquete rosserial. 
* Abre una terminal en Ubuntu Mate.

* Instala la librería de rosserial con los siguientes comandos:

```
sudo apt-get install ros-melodic-rosserial
```

```
sudo apt-get install ros-melodic-rosserial-arduino
```
---

### 2.3 Encontrar el puerto USB.
* Conecta el Arduino a tu computadora mediante un cable USB.

* Abre una terminal en Ubuntu Mate.

* Escribe el siguiente comando en la terminal para mostrar una lista de los puertos seriales disponibles:

```
ls /dev/tty*
```

Esto mostrará una lista de todos los dispositivos de puerto serie disponibles en tu sistema, incluyendo el puertos al  que está conectado el Arduino.

* Busca el nombre del puerto en la lista que se muestra. Por lo general, el puerto que usa el Arduino suele tener un nombre similar a "/dev/ttyACM0" o "/dev/ttyUSB0". Si tienes más de un dispositivo conectado a tu computadora, es posible que debas desconectar los otros dispositivos para determinar qué puerto está utilizando el Arduino.

* Ahora que has identificado el nombre del puerto en el que está conectado el Arduino, puedes usar ese nombre para el siguiente paso.

---
### 2.4 Agregar permisos.
Agregamos los permisos para poder acceder a los puertos USB:
```
sudo usermod -a -G dialout tu_usuario
```


### 2.5 Prueba de comunicación.
Una vez realizado lo anterior, podemos solicitar el tópico al arduino con la siguiente linea de comando:
```
rosrun rosserial_python serial_node.py "puerto"
```
La terminal mostrará si se ha establecido comunicación con el Arduino y qué tipo de mensaje se está recibiendo de éste.

Para escuchar el topico publicado:
```
rostopic echo /tu_tópico
```
---
## 3. Videotutorial de la sección.
[![](https://markdown-videos.deta.dev/youtube/OBoX4kjMfzA)](https://youtu.be/OBoX4kjMfzA)
