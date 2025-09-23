import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # --- PARÂMETROS DE SINTONIA FINA BASEADOS NOS SEUS LIMITES ---
    
    # Limite X de +/- 0.12 -> Usamos 0.10 para segurança
    passo_max = 0.12
    
    # Limite Z superior de +0.06 -> Usamos 0.05 para segurança
    altura_passo = 0.06
    
    # Limite Z inferior de -0.04 -> Usamos 0.01 (1cm abaixo do neutro), bem seguro
    profundidade_stance = 0.03

    # Parâmetros de comportamento (pode manter por enquanto)
    freq_base = 1.0
    freq_ganho = 2.0
    ganho_amplitude = 20.0
    forca_acoplamento = 10.0

    ld = LaunchDescription()

    cpg_node = Node(
        package='caramel_cpg',
        executable='cpg_node',
        name='cpg_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'passo_max': passo_max},
            {'altura_passo': altura_passo},
            {'profundidade_stance': profundidade_stance},
            {'freq_base': freq_base},
            {'freq_ganho': freq_ganho},
            {'ganho_amplitude': ganho_amplitude},
            {'forca_acoplamento': forca_acoplamento},
        ]
    )

    ld.add_action(cpg_node)

    return ld