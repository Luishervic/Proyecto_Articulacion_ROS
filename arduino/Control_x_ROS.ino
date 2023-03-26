#include <SimpleFOC.h> // Incluimos la librería SimpleFOC.
#include <ros.h> // Incluimos la librería ROS.
#include <sensor_msgs/JointState.h> // Incluimos la librería JointState.

ros::NodeHandle  nodo; //Creamos el nodo.

//---------------------Instanciamos los elementos del hardware---------------------
MagneticSensorSPI sensor = MagneticSensorSPI(48, 14, 0x3FFF);
// MagneticSensorSPI(int cs, float resolución, int registro_angulo)
// Instanciamos el objeto sensor Magnético con los siguientes parámetros:
// cs - Pin de selección del chip SPI conectado al Arduino.
// bit_resolution - Resolución del sensor magnético.
// angle_register - Registro de lectura de ángulo. (Consultar el manual del fabricante :D)

BLDCMotor motor = BLDCMotor(8,0.066,1320);
// BLDCMotor(int pp, int Ω, int KV)
// - pp             - número de pares de polos del motor.
// - Ω              - resistencia de fase del motor.
// - KV             - RPM/volt del motor. (Consultar el manual del fabricante :D)

BLDCDriver3PWM driver = BLDCDriver3PWM(11,9,10,6,3,5);
// BLDCDriver3PWM(int phA, int phB, int phC, int enA, int enB, int enC)
// - phA, phB, phC - pines PWM de fase A, B y C.
// - enA, enB, enC - pin de habilitación para cada fase.

float joint_pos = 0.0; 
void jointStateCallback(const sensor_msgs::JointState& msg) {
  joint_pos = (msg.position[0]*100);  // Asignamos el valor de la junta a la variable joint_pos.
}
// Configuramos el suscriptor.
ros::Subscriber<sensor_msgs::JointState> jointStateSub("/joint_states", jointStateCallback);

void setup(){
  // -------------------- ROS --------------------//
  nodo.getHardware()->setBaud(115200); // Velocidad de comunicación Arduino - ROS.
  nodo.initNode(); //Inicializamos el nodo.
  nodo.subscribe(jointStateSub);
  
  // -------------------- SimpleFOC --------------------//

  // Inicializamos el sensor mágnetico, y lo asociamos al motor.
  sensor.init();
  motor.linkSensor(&sensor);
  
  //Configuramos la frecuencia a la que trabajará el pwm [Hz].
  driver.pwm_frequency = 2500;
  // Configuramos la velocidad del puerto de monitoreo [bps].
  Serial.begin(115200);
  // Configuramos la velocidad del puerto de control [bps].
  //
  
  //Serial1.begin(115200);
  // Voltaje de la fuente de poder [V].
  driver.voltage_power_supply = 12;
  
  // Iniciamos el driver.
  driver.init();
  motor.linkDriver(&driver);

  //Ingresamos el limite de corriente y de velocidad.
  motor.current_limit = 15; // [Amperios]
  motor.velocity_limit = 20; // [rad/s]

  // Configuramos el tipo de control en este caso, control de ángulo.
  motor.controller = MotionControlType::angle;

  // Configuración del PI de velocidad.
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;

  // Configuración del control del ángulo. 
  motor.P_angle.P = 1;
  // Se inicializa el motor.
  motor.init();
  // Se inicializa el control FOC del motor.
  motor.initFOC();
  // Setpoint de la posición inicial.
  motor.target = 0;
  // Activamos el monitoreo del motor en el puerto Serial1.
  motor.useMonitoring(Serial1);
  // Variables a observar: Setpoint y ángulo.
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE;
  Serial1.println("Configuracion terminada");
  _delay(1000);
}

void loop(){
  // Función principal del algoritmo FOC.
  motor.loopFOC();
  // Función de control de movimiento..
  motor.move(joint_pos);
  // Comunicación con el usuario.
  motor.monitor();
  nodo.spinOnce();
}
