#ifndef NMPC_HPP
#define NMPC_HPP

#include <vector>
#include <map>
#include <string>
#include <casadi/casadi.hpp> // Inclua a biblioteca CasADi

class NMPCController {
public:
    // Construtor
    NMPCController(
        double L = 0.15,        // Distância entre rodas (m)
        double dt = 0.1,        // Intervalo de tempo (s)
        int N_pred = 10,        // Horizonte de previsão
        double max_delta_rad = 40.0 * M_PI / 180.0, // Delta máximo em radianos
        double Q_offset = 2.5, // Peso para erro lateral
        double Q_psi = 0.50,   // Peso para erro de orientação
        double R_delta_rate = 0.2 // Peso para taxa de mudança de delta
    );

    // Função para calcular o ângulo de controle delta
    // offset_m: erro lateral em metros
    // psi_rad: erro de orientação em radianos
    // current_velocity_mps: velocidade atual do veículo em m/s
    // O parâmetro 'current_theta_rad' foi removido, pois não é usado na lógica do MPC.
    double computeControl(double offset_m, double psi_rad, double current_velocity_mps);

private:
    double L; // Distância entre rodas
    double dt; // Intervalo de tempo
    int N_pred; // Horizonte de previsão

    // Pesos da função custo (reordenados para corresponder ao construtor)
    double Q_offset;
    double Q_psi;
    double R_delta_rate;

    double max_delta_rad; // Delta máximo em radianos

    casadi::Function solver; // Objeto solver do CasADi
    std::map<std::string, casadi::DM> arg; // Argumentos para o solver
    std::map<std::string, casadi::DM> res; // Resultados do solver

    double prev_delta; // Armazena o delta calculado no passo anterior para penalizar a taxa de mudança
    double prev_x, prev_y, prev_theta; // Estado inicial da última otimização (prev_x é o mais relevante para warm-start)

    // Método para inicializar o problema de otimização do CasADi
    void setupCasADiProblem();
};

#endif // NMPC_HPP
