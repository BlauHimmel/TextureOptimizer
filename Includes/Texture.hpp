#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include <cstdint>
#include <string>
#include <memory>

// The first pixel pointed to is top-left-most in the image
class Texture
{
public:
    Texture() = default;
    Texture(const Texture & other) = delete;
    Texture(const Texture && other) = delete;

    bool Create(uint32_t width, uint32_t height, uint32_t channel);
    bool Load(const std::string & filename);
    bool Write(const std::string & filename) const;

    std::unique_ptr<Texture> Copy() const;
    void CopyTo(Texture * pTexture) const;

    uint8_t & At(uint32_t x, uint32_t y, uint32_t channels) const;
    uint8_t * At(uint32_t x, uint32_t y) const;
    std::unique_ptr<uint8_t[]> Sample(double x, double y) const;

    bool IsValid() const;
    uint32_t Width() const;
    uint32_t Height() const;
    uint32_t Channels() const;

    static std::unique_ptr<Texture> CreateChessBoard(uint32_t width, uint32_t height, uint32_t xCount, uint32_t yCount);

private:
    static uint8_t * Fetch(const Texture * pTexture, uint32_t x, uint32_t y, uint32_t channels);

private:
    bool m_bValid = false;
    uint32_t m_Width = 0;
    uint32_t m_Height = 0;
    uint32_t m_Channels = 0;
    std::unique_ptr<uint8_t[]> m_Data;
};

#endif //TEXTURE_HPP
