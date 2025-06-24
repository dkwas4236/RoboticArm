from stockfish import Stockfish

stockfish = Stockfish("/usr/games/stockfish") 

stockfish.set_skill_level(20)  
stockfish.set_depth(25)  

# Set the starting position
start_fen = "rnbqkbnr/pppp1ppp/8/4p3/6P1/5P2/PPPPP2P/RNBQKBNR b KQkq - 0 2"
stockfish.set_fen_position(start_fen)

best_move = stockfish.get_best_move()

print("Black's best move:", best_move)

