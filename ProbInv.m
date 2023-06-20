% Script para resolver el problema INVERSO en un robot Scara usando el 
% Robotic Toolbox de Peter Corke que permite crear un robot a partir de 
% los parámetros de DH.

% Antes de nada definimos los parámetros fijos definidos por el fabricante
% del robot.

d1 = 387;
a1 = 325;
a2 = 275;

% En el problema inverso el objetivo es calcular el valos que tienen que
% adoptar las variables para que el en-deffector llegue a una posción dada. 
% Definammos dicha posición
%FALTA COMPROBAR LIMITACIONESSSS

x = -200;
y = -200;
z = 300;


% Resolvemos el problema inverso según la posición asiganda al 
% end-deffector,es decir, calculamos el valor de las variables 
% (theta1, theta2, d3 y theta4)

% Antes de nada vamos a comprobar si las posiciones definidas son posibles
% de alcanzar con el robot. 

D = (x^2+y^2-a1^2-a2^2)/(2*a1*a2)

if D^2 > 1
    disp('Punto fuera del espacio de trabajo del robot Scara')
else 
% Cálculo theta2 en rad:
th2 = atan2(-sqrt(1-D^2),D) %estoy considerando el signo + de la raiz 
                            % habria otra solución conm el signo -
th_2 = th2*180/pi %paso a grados
    if  th_2 <-150 || th_2 > 150
    disp('Punto fuera del espacio de trabajo del robot Scara')
    else 
    % Cálculo theta1 en rad:
    th1 = atan2(y,x)-atan2(a2*sin(th2),a1+a2*cos(th2))
    th_1 = th1*180/pi
        if th_1 <-105 ||  th_1 > 105
        disp('Punto fuera del espacio de trabajo del robot Scara')
        else
        % Cálculo d3:
         d3 = d1-z
            if d3 > 210
            disp('Punto fuera del espacio de trabajo del robot Scara')
            else 
            disp('Punto dentro del espacio de trabajo del robot Scara')

% En el caso de que el punto esté dentro del espacio de trabajo del robot 
% procedemos a resolver el problema inverso y a representar la trayectoria 
% del robot

% Creamos las articulaciones  mediante la función Link. Esta
% función crea las articulaciones usando los paramétros de DH. Sólo tiene 3
% entradas ya que el parámetro que no se incluye lo asume como variable, y
% en función de si este parametro variable es theta o d, la articulación
% será revolute o prismatic. En el caso del par prismático hay que
% espicificar sus valores límite. 

L1 = Link('d',d1,'a',a1,'alpha',0,'qlim',[-105*pi/180,105*pi/180]); %R
L2 = Link('d',0,'a',a2,'alpha',pi,'qlim',[-157,5*pi/180,157,5*pi/180]); %R
L3 = Link('theta',0,'a',0,'alpha',0,'qlim',[0,210]); %P 
L4 = Link('d',0,'a',0,'alpha',0); %R no tiene limitación de giro

% LOS LÍMITES NO FUNCIONAN BIEN 

% Armamos el robot con la función SerialLink que crea la cadena cinemática.

scara = SerialLink([L1,L2,L3,L4],'name','Scara');


% Definimos una posición de partida del robot pos_offset(th1,th2,d3,th4)
% que se corresponde con la posición de reposo:

pos_offset = [0 0 0 0];

% En el caso de que el robot parta de otra posición ya definida (este sería
% el caso de varias simulaciones) su posición inicial será esa, en caso
% contrario la posición inicial se corresponderá con la pos_offset.

if exist ('pos_initial') == 0 % en el caso de que no exista pos inicial definida
    pos_initial = pos_offset;
end

% En el caso del problema inverso, para poder representar el robot en las 
% sucesivas posiciones hasta llegar a la posición final deseada, la asiganda,
% primero habrá que determinar la trayectoria seguida por el end-deffector
% y después calculatr el valor de las variables del robot correspondiente 
% a cada sucesiva posición del end deffector.

% PASO 1: Cálculo posiciones sucesivas del end-deffector
% Vamos a definir 3 vectores cada uno asociado a una coordenada (x,y,z) de 
% tal manera que cada uno de ellos contenga la coordenada correspondiente
% de todos los puntos de la trayectoria seguida por el end deffector
% hasta llegar a su posición final.
 
% Antes de nada tengo que calcular la posición inicial del end-deffector, 
% es decir, he de de resolver el problema directo para la poisición 
% inicial del robot.

x0 = a1*cos(pos_initial(1)) + a2*cos(pos_initial(1)+pos_initial(2));
y0 = a1*sin(pos_initial(1)) + a2*sin(pos_initial(1)+pos_initial(2));
z0 = d1 - pos_initial(3);

% Creamos los vectores asociados a cada coordenada usando la función 
% linspace(x1,x2,N). Esta función nos proporciona un vector fila con N 
% puntos igualmente espaciados que van desde X1 a X2, en nuestro caso 
% desde la coordenada inicial (x0, y0 o z0) hasta el valor 
% final que hayamos introducido (x,y,z).

x_path = linspace(x0,x,20);
y_path = linspace(y0,y,20);
z_path = linspace(z0,z,20);

% PASO 2: Determinación de los sucesivos valores de las VAR
% Vamos a definir una matriz Var_path 20x4 ( 4 columnas: th1,th2,d3,th4 / 
% 20 filas para los 20 puntos d ela trayectoria del end-deffector) que
% contenga todos los valores de las VAR del robot para llegar desde su 
% posición inicial hasta la asignada. Para obtener las VAR correspondientes
% a cada punto habrá que resolver el problema inverso en cada punto.
% Para ello, utilizaremos un ciclo for.

% Guardamos espacio para Var_path inicializando la matriz con todo ceros:

Var_path = zeros(20,4);

for i = 1:1:20 % ciclo for que va desde 1 hasta 20 con incrementos de 1 
    D = ((x_path(i))^2+(y_path(i))^2-a1^2-a2^2)/(2*a1*a2);
    Var_path(i,2) = atan2(sqrt(1-D^2),D); %th2

    Var_path(i,1) = atan2(y_path(i),x_path(i))-atan2(a2*sin(Var_path(i,2)),a1+a2*cos(Var_path(i,2))); %th1
            
    Var_path(i,3) = d1-z_path(i); %d3

end

% Cómo calculo th4????? -> hago que no cambie?????
Var_path(:,4) = linspace(pos_initial(4),pos_initial(4),20);


% Una vez calculados tanto los puntos de las trayactoria del 
% end -deffector como las variables asociadas a cada punto procedemos a
% representar el robot.

% Primero, represento la trayectoria seguida por el end-deffector. 
% Al tratarse de una trayectoria en 3D ( ejes x,y,z) utilizaremos la 
% función plot3.Pero antes de nada eliminaremos la trayectoria del
% end-deffector de simulaciones previas si es que existe y el punto de la
% previa simulación.

if exist ('trayecEndDef') == 1 % en el caso de que no exista pos inicial definida
    delete(trayecEndDef)
elseif exist('point') == 1
    delete('point')
end

trayecEndDef = plot3(x_path,y_path,z_path,'Color',[1 0 0], 'LineWidth',2);

hold on

% definimos los límites del plot

xlim([-400,1000]);
ylim([-600,600]);
zlim([0,1000]);

% A continuación, represento el punto al que el usuario quiere mover el
% end-deffector usando una esfera amarilla:

   point = plot_sphere([x,y,z], 50, 'y'); 

% Finalemnete, procedemos a representar el robot en las sucesivas 
% posiciones hasta llegar a la posición asignada (esfera amarilla). 
% Para ello, al igual que hemos hecho con la trayectoria seguida por el 
% end-deffector emplearemos un bucle for.

for i=1:20
    scara.plot([Var_path(i,1) Var_path(i,2) Var_path(i,3) Var_path(i,4)], ...
        'scale',0.5,'perspective','jointdiam',2,'jaxes','shadow');
end


% Guardo la posición final del robot, el valor final de mis variables, para
% que en la siguiente simulación esa sea la posición inicial del robot.

pos_initial = [th1 th2 d3 Var_path(20,4)]; 
            end
        end
    end
end