function [ X ] = EliminacaoGaussPP( matriz_pesos, matriz_saidas )
    %Implementação do método da eliminação Gaussiana
    linhas = size(matriz_pesos,1);
    colunas = size(matriz_pesos,2);
    X = zeros(linhas,1);
    if linhas ~= colunas
        disp('A matriz de pesos deve ser quadrada'); 
    else
        if size(matriz_saidas,1) < size(matriz_saidas,2)
           matriz_saidas = matriz_saidas';
        end
        if (size(matriz_saidas,1) ~= linhas)||(size(matriz_saidas,2)~=1)
           disp('A matriz de saída deve ter uma dimensão com o mesmo tamanho da matriz de pesos e a outra unitária'); 
        else    
            matriz_pesos = [matriz_pesos matriz_saidas];
            tam = colunas+1;
            %Transforma a matriz de pesos em uma matriz triangular superior
            for i = 2:linhas
                %pivoteamento parcial
                [M I] = max(abs(matriz_pesos(i-1:linhas,i-1)));
                I = I + (i-2);
                M = matriz_pesos(I,i-1:tam);
                matriz_pesos(I,i-1:tam) = matriz_pesos(i-1,i-1:tam);
                matriz_pesos(i-1,i-1:tam) = M;
                
                p = matriz_pesos(i:linhas,i-1)./matriz_pesos(i-1,i-1);
                %subsitui a coluna i-1 abaixo da diagonal por 0
                %Lx = Lx - a/b*Li é aplicada apenas às demais colunas
                %isso visa maior eficiência do algoritmo, evitando operações a
                %mais de multiplicação e substração
                sub = p*matriz_pesos(i-1,i:tam);
                matriz_pesos(i:linhas,i-1) = zeros(linhas-i+1,1); 
                matriz_pesos(i:linhas,i:tam) = matriz_pesos(i:linhas,i:tam) - sub; 
            end
            %Resolve a partir da matriz triangular superior
            X(linhas,1) = matriz_pesos(linhas,tam)./matriz_pesos(linhas,colunas);
            for i = 1:linhas-1
                k = linhas-i;
                X(k,1) = (matriz_pesos(k,tam) - matriz_pesos(k,k+1:colunas)*X(k+1:linhas,1))./matriz_pesos(k,k);
            end    
        end    
    end    

end

