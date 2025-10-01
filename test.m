% グラフ表示の例

% x軸の値を生成（-5から5まで、0.1刻み）
x = -5:0.1:5;

% サブプロット1: 正弦関数
subplot(3,1,1)
y1 = sin(x);
plot(x, y1, 'b-', 'LineWidth', 2)
title('正弦関数: sin(x)')
grid on

% サブプロット2: 二次関数
subplot(3,1,2)
y2 = x.^2;
plot(x, y2, 'r-', 'LineWidth', 2)
title('二次関数: x^2')
grid on

% サブプロット3: 指数関数
subplot(3,1,3)
y3 = exp(x);
plot(x, y3, 'g-', 'LineWidth', 2)
title('指数関数: e^x')
grid on

% グラフ全体の設定
sgtitle('基本的な数学関数のグラフ')