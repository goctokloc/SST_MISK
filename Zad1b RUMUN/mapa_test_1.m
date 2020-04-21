%współrzędne stacji wyrażone w metrach (x,y) o początku układu
%współrzędnych po lewej na dole(wypelnianie jest macierzą od lewej od
%góry!!!)

p=zeros(1000,1000);
p(1:1000,1:10)=ones(1,1);
p(1:10,1:1000)=ones(1,1);
p(1:1000,990:1000)=ones(1,1);
p(990:1000,1:1000)=ones(1,1);
%p(75:125,75:125)=ones(1,1);%stacja(1,9)
p(50:100,250:300)=ones(1,1); %przeszkoda
p(1:50,700:750)=ones(1,1); %przeszkoda
%p(75:125,875:925)=ones(1,1);%stacja(9,9)
p(200:300,100:400)=ones(1,1); %przeszkoda
p(200:300,550:850)=ones(1,1); %przeszkoda
p(400:450,250:450)=ones(1,1); %przeszkoda
p(400:600,650:700)=ones(1,1); %przeszkoda
p(400:500,850:950)=ones(1,1); %przeszkoda
%p(475:525,475:525)=ones(1,1); %STACJA BAZOWA(5,5)
p(550:600,100:150)=ones(1,1); %przeszkoda
p(700:900,100:400)=ones(1,1); %przeszkoda
%p(625:675,225:275)=ones(1,1); %stacja(2.5,3.5)
p(800:850,500:550)=ones(1,1); %przeszkoda
p(900:950,600:650)=ones(1,1); %przeszkoda
p(650:700,600:650)=ones(1,1); %przeszkoda
%p(775:825,625:675)=ones(1,1); %stacja(6.5,2)
p(650:950,750:850)=ones(1,1); %przeszkoda
%p(750:800,900:950)=ones(1,1); %stacja(9.25,2.25)


map = binaryOccupancyMap(p,100);
inflate(map,0.25)
%show(map)

