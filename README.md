# Команда Кабинетный Болид
![General](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Превью.jpg)

### Робот, созданный для решения задачи РРО Будущие инженеры 2023
### © Беззубцев Федор, Федоров Тимофей 

## Наши фотографии
  [Обычная фотография](https://github.com/Timofei1412/Cabinet-car/blob/main/team_photo/Фотграфия%20команды.jpg)
  [Смешная фотография](https://github.com/Timofei1412/Cabinet-car/blob/main/team_photo/Смешная%20фотография.jpg)
<br>

## Видео заезда нашего робота на [Youtube](https://www.youtube.com/watch?v=HyZGGjBNKJ4&ab_channel=qZer0)
<br>

## Предисловие
  > Робот является автомобилем с передними поворотными колесами и задним приводом. На роботе установлены: лидар, для определения стенок и кубиков, на подпружиненной основе (для более удобной настройки уровня лидара), камерой для определения цвета кубиков и энкодер на ведущем двигателе, для определения пройденного расстояния. Система курсового управления автомобиля управляется сервомотором. При проектировании были учитаны углы Аккермана и принцип трапеции Жанто. Во главе робота находится микроконтроллер ESP32.
  
## Содержание репозитория
  - Папка "Robot models" содержит все модели старой версии робота
  - Папка "Code" содержит код робота
  - Папка "schematic" содержит электромеханическую схему робота
  - Папка "team_photo" содержит два фото команды по регламенту
  - Папка "robot_photo" содержит обзорные фото робота с шести сторон
  - Папка "readme_photo" содержит фото, используемые в README.md файле

## Фото робота
  Здесь представлены фото робота спереди, сзади, справа, слева, сверху и снизу.
  ![Front](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Вид%20спереди.jpg)
  ![Back](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Вид%20сзади.jpg)
  ![Right](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Вид%20справа.jpg)
  ![Left](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Вид%20слева.jpg)
  ![Top](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Вид%20сверху.jpg)
  ![Bottom](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Вид%20снизу.jpg)
  
  *Если картинки не открываются, проверьте папку "robot_photo", пожалуйста
  
## Подсистема перемещения робота
  Система перемещения робота была полностью спроектированна и собранна участниками команды. Робот использует детали, напечатанные на 3D принтере. Для соединений деталей используются винты и гайки М2 и М3. В креплении дифференциала и в рулевом управлении использованы подшипники и латунные втулки, уменьшающие трение.
  
  Для движения робота вперёд используется коллекторный двигатель Polulu с редуктором. Задние колёса соеденены с мотором при помощи дифференциала, который позволяет роботу правильно проходить повороты. Дифференциал закреплён к корпусу робота. Для контроля направления движения робота используется сервомотор MG90S с металлическим редуктором, а также система тяг и кулаков. Сервомотор вращает качалку, которая в свою очередь тянет тяги, которые врашают кулаки на которых закреплены передние колёса колёса. Для связи тяг и кулаков используется система из запресованных в пластик подшипников, подшипники соеденены между собой винтом. При расчёте системы рулевого управления использовались углы Аккермана.
### Фотографии системы рулевого управления
  ![Система](https://github.com/Timofei1412/Cabinet-car/blob/main/robot_photo/Система%20рулевого%20управления.jpg)

## О наших электрических и механических компонентах
![About](https://github.com/Timofei1412/Cabinet-car/blob/main/readme_photo/about(1).png)
### Лидар
  В нашем роботе используется LD19. Он основан на принципе лазерной триангуляции расстояния и использует высокоскоростной процессор для сбора и обработки данных. Система измеряет данные о расстоянии более 4500 раз в секунду. Дальномер LD19 вращается по часовой стрелке и выполняет 360-градусное лазерное сканирование окружающего пространства. Полученные данные преобразуются главным микроконтроллером для построения виртуальных стен. Данный лидар имелся в наличии и является достаточно простым для использования, поэтому мы решили использовать именно его.

### Камера
  Для определения цвета знаков было решено использовать модуль технического зрения TrackingCam. Он является сенсорным устройством для исследования окружающего пространства путем обработки и анализа изображения со встроенной видеокамеры. Данная камера проста в настройке и использовании.

### Драйвер двигателя
   В качестве драйвера двигателя используется модуль MOSFET транзистора D4184. Данный модуль неприхотлив, прост в использовании. Однако у данного модуля есть один существенный минус: двигатель, под управлением данного драйвера, не имеет возможности двигаться в обратном направлении.

### Металлический мотор-редуктор Polulu
  Этот мотор-редуктор представляет собой миниатюрный двигатель постоянного тока средней мощности с обмоткой на 6 В и металлическим редуктором. Его поперечное сечение составляет 10 × 12 мм, а D-образный выходной вал редуктора имеет длину 9 мм и диаметр 3 мм.
  Эти миниатюрные щеточные моторы-редукторы постоянного тока выпускаются в широком диапазоне передаточных чисел - от 5:1 до 1000:1 - и с пятью различными двигателями: мощными 6 В и 12 В с угольными щетками с длительным сроком службы (HPCB), а также двигателями высокой (HP), средней (MP) и малой (LP) мощности 6 В с щетками из драгоценных металлов с меньшим сроком службы. Двигатели HPCB на 6 В и 12 В обладают одинаковой производительностью при соответствующих номинальных напряжениях, только двигатель на 12 В потребляет вдвое меньше тока, чем двигатель на 6 В. В нашем роботе используется двигатель 50:1 HPCB на 6 В.
  
### Магнитный энкодер Polulu
  На роботе команды "Кабинетный болид" закреплён магнитный энкодер polulu. На валу мотора закреплён диск магнитная полярность которого различна в разных точках. Энкодер сосоит из двух датчиков хола смещённых на 90 градусов друг относительно друга. Мы получаем с энкодера два сигнала смены магнитной полярности смещённые на 90 градусов. Это позволяет точно определить скорость вращения мотора и поддерживать её постоянной на протежение всего заезда. Данный энкодер был специально спроектирован для линейки двигателей polulu и использовать другой(сторонний) энкодер было не целесообразно 

### Датчик линии QTR-01A
  На роботе установлены четыре датчика линии QTR-01A. На основе данных датчиков мы создали свои кастомные энкодеры на ведущие колеса. Таким образом мы собираемся коректировать ошибку лидара при движении робота.

### Потенциометр
  Потенциометр на нашем роботе встроен в дно и в будеющем поможет(всесте с энкодерами на задних ведущих колесах) более точно корректировать показания лидара.
### Сервопривод MG90S
  MG90S - это маленький сервопривод похожий на популярную версию MG90. Так почему бы не использовать его синий аналог? Ответ прост - металлический редуктор. Благодаря ему сервопривод может приложить большое усилие для поворота колес без каких-либо последствий. С помощью этого сервопривода робот может довольно точно задать угол поворота колес. 
### ESP32
  Плата Lolin D32 основана на микроконтроллере ESP32 от "Espressif systems" с низким энергопотреблением. Она обладает множеством мощных функций, включая двухъядерный процессор на базе Arm, Wi-Fi, Bluetooth, I2C, I2S, SPI, АЦП, ЦАП и 4 Мб флэш-памяти. Модуль D32 предоставляет эти функции в удобном формате DIP и может быть запрограммирован непосредственно с USB-интерфейса - дополнительное оборудование для программирования не требуется.
  
### Печатная плата
  На печатной плате расположен контроллер ESP32 с необходимой обвязкой и программатором. Также печатная плата включает в себя понижающий преобразователь до 5V для питания камеры, лидара и серы, понижающий преобразователь до 3.3V для питания ESP32 и энкодера и повышающий преобразователь до 10v для питания мотора.
 
## Подсистема питания и сенсоры робота 

### Питание
  Источником питания робота является литий-полимерная батарея с номинальным напряжением 8.7 вольта. Для полного отключения питания робота, на плате размещен тумблер. На плате установлен делитель напряжения который позволяет контроллеру безопасно считать текущее напряжение акумулятора и предотвратить его разряд. На плате напряжение с акумулятора понижается до 5в при помощи схемы на основе модуля [MP1584](https://www.ozon.ru/product/ponizhayushchiy-preobrazovatel-napryazheniya-gsmin-mp1584en-dc-dc-zelenyy-285604865/?asb=uujFsyWmM5ZedbAN1P4vUE4Ul%252BvjDnRlTe9Ct5nkEX4%253D&asb2=7DaiogtcmMVU4mrKaqHwJffn-PwBVsE-z86YZYDSHlCm7M0cLorGFQkAnXwPXJMA&avtc=1&avte=4&avts=1687894174&keywords=mp1584en&sh=leogiH1JgA) для подачи его на лидар серву и камеру, ESP32. Для подачи на драйвер двигателя напряжение с акумулятора повышается до 8в при помощи схемы на основе модуля [LM2577](https://ampero.ru/lm2577-voltage-regulator-povyshayuschiy-dc-dc-preobrazovatel-3v-34v-v-4v-35v.html). 
  
### Сенсоры
  Основным сенсором для определения стенок и кубиков является лидар, он позволяет узнавать расстояние до 300 точек вокруг робота с частотой обновления 15 раз в секунду. Поскольку лидар не имет возможности чётко определять цвет объекта перед роботом, для этой цели используется камера TrackingCam.

  
## Схема электромеханического устройства
  В этом блоке представлены фотографии общей схемы робота, а также схемы основной печатной платы робота. Файл схемы печатной платы можно найти [в этой папке](https://github.com/Timofei1412/Cabinet-car/blob/main/schematic).
  ![Общая схема](https://github.com/Timofei1412/Cabinet-car/blob/main/schematic/Electric%20scheme.jpg)
  
  *Если картинка не открывается или вам нужна картинка лучшего качества, проверьте папку "schematic", пожалуйста.


## Сборка робота
  1) Необходимо напечатать все пластиковые части на 3D принтере. Для этого зайдите в папку "3D-models" и распечатайте все модели на 3D-принтере.
  2) Необходимо купить такие компоненты как: [Esp32 Lolin32](https://compacttool.ru/plata-razrabotchika-wemos-lolin32-esp-wroom-32-bluetooth-btblewifi), [Понижающий преобразователь](https://www.ozon.ru/product/ponizhayushchiy-preobrazovatel-napryazheniya-gsmin-mp1584en-dc-dc-zelenyy-285604865/?asb=uujFsyWmM5ZedbAN1P4vUE4Ul%252BvjDnRlTe9Ct5nkEX4%253D&asb2=7DaiogtcmMVU4mrKaqHwJffn-PwBVsE-z86YZYDSHlCm7M0cLorGFQkAnXwPXJMA&avtc=1&avte=4&avts=1687894174&keywords=mp1584en&sh=leogiH1JgA), [Повышающий преобразователь](https://ampero.ru/lm2577-voltage-regulator-povyshayuschiy-dc-dc-preobrazovatel-3v-34v-v-4v-35v.html), [лидар LD-19](https://aliexpress.ru/item/1005004295339153.html?sku_id=12000031808972433), [камеру TrackingCam](https://robotbaza.ru/product/modul-tehnicheskogo-zreniya-trackingcam), [мотор Polulu](https://www.pololu.com/product/3063), [сервопривод SG-90S](https://ampero.ru/mg90s-servoprivod-dlya-proektov-arduino.html), [драйвер мотора](https://www.wildberries.ru/catalog/60795780/detail.aspx). Для робота также необходимы 4 подшипника 3x9x7, 16 подшипников 2x5x2.5, 2 подшипника 10x15x6, латунные втулки 3x6x4, [дифференциал](https://rc-today.ru/product/differencial-hsp-60065/) и четыре силиконовые [шины Pololu](https://www.pololu.com/product/3408). Для питания робота можно использовать любую 7.4V 2S Li-Po батарею размером 53x30x11.5мм, например, мы используем вот эту [батарею](http://realrc.ru/product.php?id_product=416).  
  3) Для сборки напечатаных и купленных компонентов понадобится 16 винтов и гаек различных размеров(М3 /16, /10... и М2 /10):
  4) В соответствии с файлом печатной платы, спаяйте печатную плату робота (папка "schematic").
  5) Открыв 3Д файл робота в [папке](https://github.com/Timofei1412/Cabinet-car/tree/main/Robot%20models/New%20robot%20models), соберите робота, как показано на модели.


 
## Распознование стенок и препядствий
  Для распознования стенок используется алгоритм апроксимации набора точек до прямой. Робот путём геометрических вычеслений, определяет расстояние до окружающих его стенок, и высчитывает свою координату относительно одного из углов поля. Движение робота (в первоначальном виде) осуществляется при помощи системы точек, по которым робот должен проехать. Основные точки ставятся програмно, при старте робота и находятся в начале и конце каждого прямого сегмента (по середине между стенками). По нахождению кубика робот ставит в соответствущей регламенту стороне две точки: сбоку перед и сбоку за кубиком. Для распознования кубиков робот смотрит есть ли какие либо точки в местах, где должны располагатся кубики, если такие точки есть, то кубик есть, а если нет, то кубика нет. Для определения цвета кубика используются данные с камеры. Для дополнительной информации откройте код.
  ![блоксхему](https://github.com/Timofei1412/Cabinet-car/blob/main/Code/Schematic%20diagram/Schematic%20diagram.jpg)
  *Если картинка не открывается или вам нужна картинка лучшего качества, проверьте папку "Schematic diagram", пожалуйста.
  
## Установка необходимых программ и прошивка платы

  - Для программирования робота вам понадобится [Arduino IDE](https://www.arduino.cc/en/software). Это программное обеспечение с открытым исходным кодом позволяет легко писать код и загружать его в плату. Откройте exe-файл и следуйте инструкциям программы установки.

  - Установите ESP32 в менеджере плат Arduino IDE, добавив ссылку (https://dl.espressif.com/dl/package_esp32_index.json) в настройки IDE.
    - Сначала добавьте дополнительные ссылки менеджера плат
    ![Settings](readme_photo/open_settings.png)
    ![Менеджер](readme_photo/add_esp32_in_boards_manager.png)
    - Во-вторых, установите плату
    ![Boards](readme_photo/open_boards_manager.png)
    ![Install](readme_photo/install_esp32.png)
  - Используйте менеджер библиотек для установки библиотек ESP32.
  ![Открыть](readme_photo/open_library_manager.png)
    - Библиотеку для аналогового порта ESP32.
    ![AnalogWrite](readme_photo/esp32_AnalogWrite_install.png)
    - И библиотеку ESP32Servo.
    ![ESP32Servo](readme_photo/esp32_servo_install.png)
  - Выберите плату "WEMOS LOLIN32". Подключите плату с ESP32 с помощью кабеля Micro USB к компьютеру и выберите соответствующий порт в IDE arduino. Нажмите кнопку "Загрузить".
  ![Upload](readme_photo/upload_program.png)
  
  - Также вам понадобится[TrackingCam](https://appliedrobotics.ru/?page_id=633). Скачайте и откройте exe-файл и следуйте инструкциям программы установки.
  - После скачивания откройте программу и настройте камеру.
    - Подключите камеру к компьютеру и откройте программу.
    - Подберите настройки для зеленого и красного кубика(не забудьте переключить поиск(полоска снизу)на блоб два, после настройки первого кубика, иначе настройки запишутся поверх прошлых)
    - Или поставьте их как на скриншоте:
      ![Red](https://github.com/Timofei1412/Cabinet-car/blob/main/Code/Tracking%20Cam%20code/Красный%20кубик%20настройки.jpg)
      ![Green](https://github.com/Timofei1412/Cabinet-car/blob/main/Code/Tracking%20Cam%20code/Зеленый%20кубик%20настройки.jpg)
   * Настройки зависят от освещения и материала кубика. Те настройки, что указанны на скриншотах, могут не работать в вашем случае.

## Запуск робота
 - Загрузить в робота нужный код
 - Проведите жеребьевку, как указано в правилах
 - Поставьте робота на стартувую позицию
 - Включите робота переключением тумблера в замкнутое положение
 - Нажмите кнопку для запуска

