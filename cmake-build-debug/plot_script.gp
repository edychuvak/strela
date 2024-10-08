set terminal png
set output "multiple_trajectories.png"

set title "Trajectory with V^2 drag"
set xlabel "x position (m)"
set ylabel "y position (m)"

# Nastav rozsah os
set xrange [0:12]
set yrange [0:*]  # Nastav y-rozsah tak, aby zahrnoval všechny hodnoty

# Přidej kružnici na pozici (10, 0)
set object 1 circle at 10,0 radius 0.5 fillcolor rgb "red"

# Vykresli trajektorii a kruh
plot "sphere_trajectory_V2.txt" using 1:2 with lines title "Trajectory v=(v*v)", \
        "" using 1:2 with points pt 1 ps 1 lc rgb "blue",\
        "sphere_trajectory_V1.txt" using 1:2 with lines title "Trajectory v=(v)", \
        "" using 1:2 with points pt 1 ps 1 lc rgb "green",\
        "sphere_trajectory_V0.txt" using 1:2 with lines title "Trajectory v=(0)",\
        "" using 1:2 with points pt 1 ps 1 lc rgb "yellow",\