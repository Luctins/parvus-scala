#Gs1 = tf([1],[1 5 10 10 5 1]);
#data = csvread(
pkg load control
# Função de tempo de acomodação
# @author: Lucas Martins Mendes - contact@luctins.me
function ret = numeric_settle_t(X,T)
  _X = flip(X);
  _T = flip(T);
  tolerance = 0.02;
  s_top = (1+tolerance)*X(end);
  s_bot = (1-tolerance)*X(end);
  for i = 1:size(T)
    if X(i) < s_bot ||  X(i) > s_top
      ret = T(i);
    endif
   endfor
endfunction

# Função de tempo de subida para o pacote de controle
# @author: Lucas Martins Mendes - contact@luctins.me

function ret = numeric_rise_t(X,T,F)
  V_reg = X(end);
  t_start = 0;
  t_end = 0;
  v_start = 0.1*V_reg;
  v_end = 0.9*V_reg;
  for i = 1:size(T)
    if X(i) >= v_start && t_start != 0
      t_start = T(i);
    endif
    if X(i) >= v_end
      t_end = T(i);
      break;
    endif
   endfor
   ret = t_end - t_start;
endfunction

function ret = H(t) 
  if t >= 0
    ret = 0;
  else
    ret = 1;
  endif
endfunction

clear

data = csvread('step_test.csv');

t = data(2:end,1);
y = data(2:end,2);
y = y - y(2);
y = y * 0.01;

t_settle = numeric_settle_t(y,t)
t_rise = numeric_rise_t(y,t)

K_p = 2.19
G_smith = tf([K_p],[23.7 1])
G_sund = tf([K_p],[20.922 1])

[Y_smith, T_smith] = step(G_smith);
[Y_sund, T_sund] = step(G_sund);

hold on
set(gca, "linewidth", 2, "fontsize", 18)

#title("Dados da Planta")
title("Comparação dos modelos")

plot(t, y, "r", "linewidth", 2)
legend("Dados planta", "location", "southeast")

plot(T_smith,Y_smith, "b;Smith;", "linewidth", 2)
plot(T_sund, Y_sund, "g ;Sundaresan/Krishnaswamy;", "linewidth", 2)

mat = [ [T_smith] [Y_smith] ]
hold off
#grid