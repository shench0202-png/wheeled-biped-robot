%% 倒立擺系統狀態空間模型建立
clear; clc;

% 1. 定義物理參數 (請根據實際硬體數值修改)
M = 1.0;      % 台車/車身質量 (kg)
m = 0.2;      % 擺桿質量 (kg)
l = 0.3;      % 擺桿重心長度 (m)
g = 9.81;     % 重力加速度 (m/s^2)
Jb = 0.005;   % 擺桿繞重心轉動慣量 (kg*m^2)
R = 0.03;     % 輪子半徑 (m)

% 2. 根據筆記推導的中間變數 (Section 5 & 6)
Meq = M + m;            % 等效質量
Jeq = Jb + m*l^2;       % 等效轉動慣量 (筆記中為 Ml^2，若 m 指的是擺桿則同理)
Delta = Jeq * Meq - (m*l)^2; % 矩陣行列式分母

% 3. 建立狀態空間矩陣 A (狀態向量 x = [x, x_dot, theta, theta_dot]')
% 根據筆記：
% x_dot = A*x + B*u
% 矩陣 A 的第三欄係數源自線性化後的 M^-1 * [0; M*g*l*theta]
a23 = -(m^2 * l^2 * g) / Delta;
a43 = (Meq * m * l * g) / Delta;

A = [0, 1, 0, 0;
     0, 0, a23, 0;
     0, 0, 0, 1;
     0, 0, a43, 0];

% 4. 建立狀態空間矩陣 B (輸入 u = tau 扭矩)
% 根據筆記：B = [0; Jeq/(R*Delta); 0; -m*l/(R*Delta)]
b2 = Jeq / (R * Delta);
b4 = -(m * l) / (R * Delta);

B = [0; b2; 0; b4];

% 5. 定義輸出矩陣 C 與 D (假設我們觀測所有狀態)
C = eye(4); 
D = zeros(4, 1);

% 6. 建立系統模型
sys = ss(A, B, C, D);
sys.StateName = {'x', 'x_dot', 'theta', 'theta_dot'};
sys.InputName = {'torque'};

% 顯示系統矩陣
disp('狀態空間矩陣 A:'); disp(A);
disp('狀態空間矩陣 B:'); disp(B);

% 7. 檢查穩定性 (特徵值)
poles = eig(A);
disp('系統極點:'); disp(poles);

% 繪製步階響應 (未閉迴路前通常會發散，因為倒立擺不穩定)
% step(sys);