from ultralytics import YOLO
import cv2
import sys
import serial

# Função de liberar conexões
def releaseConnections(cap, ser):
    # Libera o acesso à câmera
    if cap.isOpened():
        cap.release()

    # Fecha a conexão serial
    if ser.is_open:
        ser.close()

    cv2.destroyAllWindows()


# Função para definir a posição da base
def posicao_base(ser, angle):
    try:
        ser.write(str(angle).encode())
    except:
        releaseConnections(cap, ser)
        sys.exit(1)

# Função para calcular a área da bounding box
def calculateArea(rect):
    return (rect[2] - rect[0]) * (rect[3] - rect[1])

# Função para calcular score
def calculateScore(area, conf):
    return area * conf

def calculate_base_angle(m_coord_x):
    base_angle = m_coord_x*180/600
    posicao_base(ser, base_angle)
    print(base_angle)

# Create a serial object
try:
    ser = serial.Serial("COM8", 9600, timeout=0.01)
    ser.open()
except serial.SerialException:
    print('Conectando...')


# Carrega o nosso modelo pré treinado
model = YOLO('weight-hand-segmentation-v9.pt')

# Abre o dispositivo de captura
cap = cv2.VideoCapture(0)


# Loop através dos frames
while cap.isOpened():

    # Extrai o frame e seu status de sucesso na leitura
    read, frame = cap.read()

    if read:

        # Processamento do frame usando o modelo pré treinado
        results = model(source=frame, conf=0.6)

        '''
        Extrai as coordenadas da caixa de detecção de cada mão,
        a informação de qual classe foi detectada e sua confiança
        '''
        boxes = results[0].boxes.cpu().numpy()

        # Métricas para a decisão da melhor detecção
        best_detection = None
        best_score = 0

        # Loop através de todas as detecções na imagem
        for i, box in enumerate(boxes):

            # Detecta se há pelo menos uma mão na imagem
            if box.cls in [0, 1]:

                rect_coord = box.data[0][:4]
                conf = box.data[0][4]
                area = calculateArea(rect_coord)

                # Calcula a localização do ponto central
                m_coord_x = int((box.xyxy[0][2] + box.xyxy[0][0]) / 2)
                m_coord_y = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)

                # Exibe a área
                cv2.putText(frame, f'Area {i + 1}: {area:.2f}', (int(box.xyxy[0][0]), int(box.xyxy[0][3])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                # Exibe o ponto central
                cv2.circle(frame, (m_coord_x, m_coord_y), 3, (0, 255, 0), 2)

                calculate_base_angle(m_coord_x)

                score = calculateScore(area, conf)

                # Atualiza a melhor detecção se a pontuação for maior
                if score > best_score:
                    best_score = score
                    best_detection = box

        # Verifica se alguma mão foi detectada
        if best_detection is not None:

            # Detecção da classe
            if best_detection.cls == 0:
                print("fechada")
            elif best_detection.cls == 1:
                print("aberta")

        else:
            print('No hand detection\n')

        # Plota o resultado do processamento e o exibe
        annotated_frame = results[0].plot(boxes=False)
        cv2.imshow('Camera Feed', annotated_frame)

        # Só fecha a janela se o usuário digitar a tecla 't'
        if cv2.waitKey(10) & 0xFF == ord('t'):
            break

    else:
        print('Error while reading camera feed')
        break

releaseConnections(cap, ser)