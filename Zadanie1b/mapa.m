p=zeros(200,200);
p(1:200,1)=ones(1,1);
p(1,1:200)=ones(1,1);
p(1:200,200)=ones(1,1);
p(200,1:200)=ones(1,1);
p(15:25,15:25)=ones(1,1);%stacja
p(10:20,50:60)=ones(1,1); %przeszkoda
p(1:10,140:150)=ones(1,1); %przeszkoda
p(15:25,175:185)=ones(1,1);%stacja
p(40:60,20:80)=ones(1,1); %przeszkoda
p(40:60,110:170)=ones(1,1); %przeszkoda
p(80:90,50:90)=ones(1,1); %przeszkoda
p(80:120,130:140)=ones(1,1); %przeszkoda
p(80:100,170:190)=ones(1,1); %przeszkoda
p(95:105,95:105)=ones(1,1); %STACJA BAZOWA
p(110:120,20:30)=ones(1,1); %przeszkoda
p(140:180,20:80)=ones(1,1); %przeszkoda
p(125:135,45:55)=ones(1,1); %stacja
p(160:170,100:110)=ones(1,1); %przeszkoda
p(180:190,120:130)=ones(1,1); %przeszkoda
p(130:140,120:130)=ones(1,1); %przeszkoda
p(155:165,125:135)=ones(1,1); %stacja
p(130:190,150:170)=ones(1,1); %przeszkoda
p(150:160,180:190)=ones(1,1); %stacja

map = binaryOccupancyMap(p,10);
inflate(map, 0.25)
show(map)
