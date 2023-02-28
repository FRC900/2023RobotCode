#ifndef GAMEPIECES_H
#define GAMEPIECES_H

struct SafetyState {
  double max_angle;
  double min_angle;
  SafetyState() {
    max_angle = 0;
    min_angle = 0;
  }
  SafetyState(double above, double below) {
    max_angle = above;
    min_angle = below;
  }
};

struct PieceMode {
  int piece;
  int mode;
  bool valid;

  PieceMode(int piece, int mode) {
    if ( piece < 0 || piece > 3 || mode < 0 || mode > 3) {
      this->valid = false;
    }
    else {
      this->valid = true;
    }
    this->piece = piece;
    this->mode = mode;
  }

  PieceMode() {}

  bool operator< (PieceMode const &rhs) const {
    if (piece < rhs.piece) {
      return true;
    }
    if (piece == rhs.piece && mode < rhs.mode) {
      return true;
    }
    return false;
  }

  bool isValid() {
    return this->valid;
  }
};

constexpr std::array<const char *, 4> mode_to_string = {"INTAKE", "LOW_NODE", "MIDDLE_NODE", "HIGH_NODE"};
constexpr std::array<const char *, 4> piece_to_string = {"CUBE", "VERTICAL_CONE", "BASE_TOWARDS_US_CONE", "BASE_AWAY_US_CONE"};

#endif // gamepieces_h
