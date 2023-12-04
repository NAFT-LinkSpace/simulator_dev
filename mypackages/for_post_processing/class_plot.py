#審査書用に地図を作るプログラムをクラス化しました。Plotクラスにはname = でresultディレクトリにあるディレクトリの名前をstrで渡して下さい。
#インスタンスを作成して,Plot(self.name).plot()で"map_pictures"フォルダに保存する。

import pyproj
import numpy as np
import pandas as pd
from math import log, tan, pi, cos
import math
from PIL import ImageDraw, ImageFont
from staticmap import StaticMap
import glob
import os
#構造
#class Plot
#|プロット
#|- class Calc　関数
#


##################global variables#########################
MIN_WIND = 6
NUMBER_OF_ANGLE = 8
NUMBER_OF_WIND = 1
LAUNCH_LON = 140.009121
LAUNCH_LAT = 40.237674
LAUNCHER_AZIMUTH_DEG = 270
MAP = StaticMap(3000, 3000, url_template='https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png')
PIC = MAP.render(zoom= 16, center=[LAUNCH_LON, LAUNCH_LAT])
############################################################


#get all files
files = glob.glob(".\\result\\*")
for file in files:
    print(file)
    
#mkdir
#os.mkdir("map_pictures")



#Outer class
class Plot:
    #Inner Class
    class Calc:
        #外部クラスで使うための関数を格納
        
        #csvからlanding pointを取得
        def get_data(self, name:str, angle:int, wind:int):
            filename = ".\\result\\" + name + "\\history_" + str(angle) + "deg_wind" + str(wind + MIN_WIND) + ".csv"
            data = pd.read_csv(filename).values
            #シミュレーター座標で、y, z座標に対応する。yは東、zは北
            return data[-1][2], data[-1][3] 
        
        #シミュレーター座標から方位角、距離を計算
        #pyprojモジュールで利用するため、時計回りを正とし、北方向を0°定義。
        def azimuth_dist(self, coord):#シミュレーター座標coord = (x, y)を渡す
            v_north = np.array([0, 1])  #北の方向
            dist = np.sqrt(np.square(coord[0]) + np.square(coord[1]))

            azimuth = np.rad2deg(math.atan2(coord[0], coord[1])) +LAUNCHER_AZIMUTH_DEG
            return azimuth, dist

        #経度座標から画像上の座標に変換。経度
        #ref WEB ブラウザ上での表示に適した配信地図データの作成技法とその応用.https://www.gsi.go.jp/common/000076319.pdf
        #ref 地理院地図から画像を取得し、任意の緯度経度に文字を入れる方法.https://blog.shikoan.com/gsi-tile/
        def lon_to_pixel(self, lon):
            # 経度→タイル番号
            if not (-180 <= lon <= 180):
                lon = (lon + 180) % 360 - 180
            x = ((lon + 180.) / 360) * pow(2, MAP.zoom)
            # タイル番号→キャンバスの座標
            pixel = (x - MAP.x_center) * MAP.tile_size + MAP.width / 2
            return pixel

        #経度座標から画像上の座標に変換。緯度
        def lat_to_pixel(self, lat):
            # 緯度→タイル番号
            if not (-90 <= lat <= 90):
                lat = (lat + 90) % 180 - 90
            y = (1 - log(tan(lat * pi / 180) + 1 / cos(lat * pi / 180)) / pi) / 2 * pow(2, MAP.zoom)
            # タイル番号→キャンバスの座標
            pixel = (y - MAP.y_center) * MAP.tile_size + MAP.height / 2
            return pixel

    def __init__(self, name:str) -> None:
        self.calc = self.Calc() #内部クラス
        self.name = name

#Calcクラスget_dataメソッドを用いて風ごとの落下分散を取得
    def get_data_list(self):	
        return  [[ self.calc.get_data(self.name, angle*45, wind) 
                for angle in range(NUMBER_OF_ANGLE)]for wind in range(MIN_WIND,NUMBER_OF_WIND)]

#Calcクラスget_azimuth_distメソッドを用いて風ごとの、更に落下地点ごとに(方位角, 距離)を取得
    def get_azimuth_dist_list(self):
        azimuth_dist = []
        for wind in range(NUMBER_OF_WIND):
            azimuth_dist_t =[]
            for angle in range(NUMBER_OF_ANGLE):
                azimuth_dist_t.append(self.calc.azimuth_dist(self.calc.get_data(self.name, angle*45, wind)))
            azimuth_dist.append(azimuth_dist_t)
        return azimuth_dist
    
#以下はGlobal変数化したので不要
#    def get_map(self):
#        #緯度経度から地図画像上の座標に変換。地図を描画
#        map = StaticMap(3000, 3000, url_template=self.map_url)
#        #緯度経度から地図画像上の座標に変換。地図を描画
#        pic = map.render(zoom= self.zoom_rate, center = [LAUNCH_LON, LAUNCH_LAT])
#        
#        return map, pic

#pyprojのgrs80でfwdで、シミュレーター座標から経度緯度に変換
    def get_lonlat_list(self):
        grs80 = pyproj.Geod(ellps = "GRS80")
        azimuth_dist = np.array(Plot(self.name).get_azimuth_dist_list())
        lon1, lat1 = [], []
        for wind in range(NUMBER_OF_WIND):
            lon1_temp, lat1_temp, back_azimuth = grs80.fwd(np.full(NUMBER_OF_ANGLE, LAUNCH_LON), np.full(NUMBER_OF_ANGLE, LAUNCH_LAT), azimuth_dist[wind, :, 0], azimuth_dist[wind, :, 1])
            lon1.append(lon1_temp)
            lat1.append(lat1_temp)
        return np.array(lon1), np.array(lat1)

#経度緯度から画像上の座標に変換。経度がｘ、緯度がｙに対応
    def get_pix_list(self):
        lon_list, lat_list = Plot(self.name).get_lonlat_list()
        pix_list = []
        for wind in range(NUMBER_OF_WIND):
            pix_list_t = []
            for angle in range(NUMBER_OF_ANGLE):
                pix_list_t.append((self.calc.lon_to_pixel(lon_list[wind, angle]), 
                                self.calc.lat_to_pixel(lat_list[wind, angle])))
            pix_list.append(pix_list_t)
        return pix_list

#落下分散をpillowで描画
    def draw_landing(self):
        draw = ImageDraw.Draw(PIC)
        x_y_pix = self.get_pix_list()
        for wind in range(NUMBER_OF_WIND):
            draw.polygon(x_y_pix[wind], outline = (int(255*wind/NUMBER_OF_WIND), 0, int(255*(1-wind/NUMBER_OF_WIND))), width=3)
        return print("landing written")

#射点をpillowで描画
    def draw_base(self):
        draw = ImageDraw.Draw(PIC)
        base_pix = ( self.calc.lon_to_pixel(LAUNCH_LON), self.calc.lat_to_pixel(LAUNCH_LAT) )
        draw.ellipse([(base_pix[0] - 3, base_pix[1] - 3), (base_pix[0] + 3, base_pix[1] + 3)], fill = "black")
        return print("base written")

#pillowのellipseを楽につかうために、円の半径と中心を与えられた時に円を書けるように簡易化
#ellipseでは円に外接する正方形の左上(top_left)と右下(bottom_right)を指定しなければならない。
#この関数では円の半径、と中心経度緯度から、外接正方形の左上、右下の画像座標を計算する。
    def __draw_circle(self, lon, lat, h_radius, c:str, w:int=4):#中心緯度、経度、半径、色(str型)
        draw = ImageDraw.Draw(PIC)
        grs80 = pyproj.Geod(ellps = "GRS80")
        #変換の計算
        lon_top_left, lat_top_left, __back_azimuth          = grs80.fwd(lon, lat, 315, h_radius* np.sqrt(2) )
        lon_bottom_right, lat_bottom_right, __back_azimuth  = grs80.fwd(lon, lat, 135, h_radius* np.sqrt(2) )
        
        pix_top_left = ( self.calc.lon_to_pixel(lon_top_left), self.calc.lat_to_pixel(lat_top_left) )
        pix_bottom_right = ( self.calc.lon_to_pixel(lon_bottom_right), self.calc.lat_to_pixel(lat_bottom_right) )
        
        draw.ellipse([pix_top_left, pix_bottom_right], fill = None, outline = c, width=w)

#同心円の目盛を描画
    def draw_index(self):
        draw = ImageDraw.Draw(PIC)

        grs80 = pyproj.Geod(ellps = "GRS80")

        index= np.arange(0, 10000, 500)

        [Plot(self.name).__draw_circle(LAUNCH_LON, LAUNCH_LAT, index_, "gray", w=2)
        for index_ in index]

        draw.line([(0, 1500), (10000, 1500)], fill = "black")
        draw.line([(1500, 0), (1500, 10000)], fill = "black")

        index_lon, index_lat, __back_azimuth = grs80.fwd(np.full(len(index), LAUNCH_LON), np.full(len(index), LAUNCH_LAT), np.full(len(index), 0), index) 

        index_pix_north = np.array([(self.calc.lon_to_pixel(index_lon[i]),self.calc.lat_to_pixel(index_lat[i]) )for i in range(len(index))])

        data = list( zip( list(map(str, index)), 
                        np.array(index_pix_north)[:,0], 
                        np.array(index_pix_north)[:,1] ) )
        # 日本語フォント
        font = ImageFont.truetype("C://Windows/Fonts/yugothb.ttc", 30)
        for point in data:
            print(point)
            # テキストを点に対して中央揃えするためにテキストのピクセルサイズを取得
            #WARNING: ImageFont.textsizeが使えなくなるらしい。
            str_w, str_h = draw.textsize(point[0], font=font)
            # 駅名の描画
            if point[1] >= 0 and point[1] < PIC.width and point[2] >= 0 and point[2] < PIC.height:
                draw.text((point[1]-str_w//2, point[2]-str_h//2 - 10), point[0], (64, 64, 255), font=font)

        return print("index written")

#すべてを描画
    def plot(self):
        self.draw_base()
        self.draw_index()
        self.draw_landing()
        return PIC.save(f".//map_pictures//lp-{self.name}.png")

#追加で多角形領域を描きたいときに使用.lonlatには経度緯度をタプルのリストで渡す[(lon, lat), ...]
    def extra_plot_polygon_area(self, lonlat):
        draw = ImageDraw.Draw(PIC)
        #経度緯度から写真上の座標に変換
        pix = [( self.calc.lon_to_pixel(lonlat[point, 0]), self.calc.lat_to_pixel(lonlat[point, 1]) ) for point in range(len(lonlat))]
        
        draw.polygon(pix, outline = "black", width= 4)
        return PIC.save(f".//map_pictures//lp-{self.name}.png") 

#追加で１点をプロットしたい時に、経度緯度を渡す
    def extra_plot_point(self, lonlat):
        draw = ImageDraw.Draw(PIC)
        #経度緯度から写真上の座標に変換
        pix = ( self.calc.lon_to_pixel(lonlat[0]), self.calc.lat_to_pixel(lonlat[1]) )
        draw.ellipse([(pix[0] - 5, pix[1] - 5), (pix[0] + 5, pix[1] + 5)], fill = "black", outline= " black")
        return PIC.save(f".//map_pictures//lp-{self.name}.png")
        
    def extra_plot_circle_area(self, lonlat, h_radius):
        draw = ImageDraw.Draw(PIC)
        #経度緯度から写真上の座標に変換
        self.__draw_circle(lonlat[0], lonlat[1], h_radius, "black")
        return PIC.save(f".//map_pictures//lp-{self.name}.png")
        
        

#以下を実行
test = Plot(name = "2023-0806-183236" )

#落下可能区域
area_lonlat = np.array([(139.950365, 40.273697), 
               (140.012675, 40.273697),
               (140.000024, 40.225331),
               (139.950365, 40.225331)])
test.extra_plot_polygon_area(area_lonlat)

#風力発電所
test.extra_plot_circle_area((139.985000, 40.248333), 150)

test.draw_index()
test.plot()
test.draw_landing()
