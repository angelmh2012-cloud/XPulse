clc 
clear 

e = zeros(16, 1);
e1 = zeros(16, 1);
e2 = zeros(16, 1)
e3 = zeros(16, 1)
e4 = zeros(16, 1)

for i= 1:3
    e(i) = 1
    e1(i + 3) = 1
    e2(i + 6) = 1
    e3(i + 9) = 1
    e4(i + 12) = 1
    
end

Ent = [e, e1, e2, e3, e4,e, e1, e2, e3, e4,e, e1, e2, e3, e4]

SalidaE1 = [200, 100, 250, 20, 0, 200, 100, 250, 20, 0, 200, 100, 250, 20, 0  ]
SalidaE2 = [0, 20, 250, 100, 200,0, 20, 250, 100, 200,0, 20, 250, 100, 200, ];;

sE = [SalidaE1 ; SalidaE2]
net = feedforwardnet(10, 'trainlm'); 

% 3. Entrenar la red
net = train(net, Ent, sE); 

% 4. Probar la red con los datos
y = net(Ent)








