function [ polinomio P ] = MMQ( x,y,n )

    linhas = size(x,1);
    colunas = size(x,2);
    if(linhas < colunas)
        x = x';
    end
    linhas = size(x,1);
    A = ones(linhas,1);
    linhasy = size(y,1);
    colunas = size(y,2);
    if(linhasy < colunas)
        y = y';
    end
    for i=1:n
        X = ones(linhas,1);
        for j=1:i
            X = X.*x;
        end
        A = [A X];
    end
    
    
    %resolve sistema matricial
    P = EliminacaoGaussPP(A'*A,A'*y);
    
    
    %escreve expressão do polinomio
    polinomio = 'P(x) = ';
    for i=1:n+1
        if i>1&P(i)>=0
            polinomio = [polinomio '+'];
        end
        polinomio = [polinomio num2str(P(i))];
        if i>1
            polinomio = [polinomio 'x'];
        end
        if i>2
            polinomio = [polinomio '^' num2str(i-1) ];
        end
    end
    

end

