from mypackages import for_main_processing
from mypackages import for_pre_processing
from mypackages import for_post_processing
from mypackages import pre_processing
#微分方程式を解く
output = for_main_processing.solve_ode_full_output.solve_all(3, 180)
#結果を出力 result dirに
for_post_processing.output_to_excel.main(output)

def main(wind_std, wind_angle):
    #微分方程式を解く
    output = for_main_processing.solve_ode_full_output.solve_all(wind_std, wind_angle)
    
    #結果を出力 result dirに
    for_post_processing.output_to_excel.main(output)