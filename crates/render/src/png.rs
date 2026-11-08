//! A PNG encoder. Deflate is written here too: LZ77 with hash chains and the fixed Huffman codes,
//! which is simple and does well on the flat colours of a rendered part.

fn crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for (n, entry) in table.iter_mut().enumerate() {
        let mut c = n as u32;
        for _ in 0..8 {
            c = if c & 1 == 1 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
        *entry = c;
    }
    table
}

pub(crate) fn crc32(table: &[u32; 256], parts: &[&[u8]]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for part in parts {
        for &b in *part {
            c = table[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
        }
    }
    c ^ 0xFFFF_FFFF
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for chunk in data.chunks(5552) {
        for &x in chunk {
            a += x as u32;
            b += a;
        }
        a %= 65521;
        b %= 65521;
    }
    (b << 16) | a
}

struct BitWriter {
    out: Vec<u8>,
    bits: u64,
    count: u32,
}

impl BitWriter {
    /// Bits go in starting from the least significant, as deflate packs its numbers.
    fn put(&mut self, value: u32, n: u32) {
        self.bits |= (value as u64) << self.count;
        self.count += n;
        while self.count >= 8 {
            self.out.push(self.bits as u8);
            self.bits >>= 8;
            self.count -= 8;
        }
    }

    /// Huffman codes go in most significant bit first.
    fn put_code(&mut self, code: u32, n: u32) {
        self.put(code.reverse_bits() >> (32 - n), n);
    }

    fn finish(mut self) -> Vec<u8> {
        if self.count > 0 {
            self.out.push(self.bits as u8);
        }
        self.out
    }
}

/// The fixed Huffman code for a literal or length symbol.
fn put_symbol(w: &mut BitWriter, s: u32) {
    match s {
        0..=143 => w.put_code(0x30 + s, 8),
        144..=255 => w.put_code(0x190 + s - 144, 9),
        256..=279 => w.put_code(s - 256, 7),
        _ => w.put_code(0xC0 + s - 280, 8),
    }
}

const LENGTH_BASE: [u16; 29] = [3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258];
const LENGTH_EXTRA: [u8; 29] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0];
const DIST_BASE: [u16; 30] = [1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577];
const DIST_EXTRA: [u8; 30] = [0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13];

/// A zlib stream holding one fixed-code deflate block.
pub(crate) fn zlib(data: &[u8]) -> Vec<u8> {
    let mut w = BitWriter { out: vec![0x78, 0x01], bits: 0, count: 0 };
    w.put(1, 1); // the last block
    w.put(1, 2); // fixed codes
    const WINDOW: usize = 32768;
    const CHAIN: usize = 16;
    let mut head = vec![usize::MAX; 1 << 15];
    let mut prev = vec![usize::MAX; WINDOW];
    let hash = |i: usize| ((data[i] as usize) << 10 ^ (data[i + 1] as usize) << 5 ^ data[i + 2] as usize) & 0x7FFF;
    let mut i = 0;
    let insert = |head: &mut Vec<usize>, prev: &mut Vec<usize>, i: usize| {
        if i + 2 < data.len() {
            let h = hash(i);
            prev[i % WINDOW] = head[h];
            head[h] = i;
        }
    };
    while i < data.len() {
        let (mut best_len, mut best_dist) = (0, 0);
        if i + 2 < data.len() {
            let mut candidate = head[hash(i)];
            let mut tries = CHAIN;
            while candidate != usize::MAX && i - candidate <= WINDOW && tries > 0 {
                let limit = (data.len() - i).min(258);
                let mut n = 0;
                while n < limit && data[candidate + n] == data[i + n] {
                    n += 1;
                }
                if n > best_len {
                    best_len = n;
                    best_dist = i - candidate;
                    if n == limit {
                        break;
                    }
                }
                let next = prev[candidate % WINDOW];
                if next != usize::MAX && next >= candidate {
                    break;
                }
                candidate = next;
                tries -= 1;
            }
        }
        if best_len >= 3 {
            let l = LENGTH_BASE.iter().rposition(|&b| b as usize <= best_len).unwrap();
            put_symbol(&mut w, 257 + l as u32);
            w.put((best_len - LENGTH_BASE[l] as usize) as u32, LENGTH_EXTRA[l] as u32);
            let d = DIST_BASE.iter().rposition(|&b| b as usize <= best_dist).unwrap();
            w.put_code(d as u32, 5);
            w.put((best_dist - DIST_BASE[d] as usize) as u32, DIST_EXTRA[d] as u32);
            // Long runs of one colour are common. Indexing every position of them buys little.
            for k in 0..best_len.min(32) {
                insert(&mut head, &mut prev, i + k);
            }
            i += best_len;
        } else {
            put_symbol(&mut w, data[i] as u32);
            insert(&mut head, &mut prev, i);
            i += 1;
        }
    }
    put_symbol(&mut w, 256);
    let mut out = w.finish();
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

/// Encodes 8-bit RGB as a PNG file.
pub fn encode_rgb(width: u32, height: u32, rgb: &[u8]) -> Vec<u8> {
    let stride = width as usize * 3;
    let mut raw = Vec::with_capacity((stride + 1) * height as usize);
    // Each line gets the filter that leaves the smallest numbers.
    let mut previous = vec![0u8; stride];
    let mut sub = vec![0u8; stride];
    let mut up = vec![0u8; stride];
    for line in rgb.chunks(stride.max(1)) {
        for i in 0..stride {
            sub[i] = line[i].wrapping_sub(if i >= 3 { line[i - 3] } else { 0 });
            up[i] = line[i].wrapping_sub(previous[i]);
        }
        let score = |v: &[u8]| v.iter().map(|&b| (b as i8).unsigned_abs() as u64).sum::<u64>();
        if score(&up) <= score(&sub) {
            raw.push(2);
            raw.extend_from_slice(&up);
        } else {
            raw.push(1);
            raw.extend_from_slice(&sub);
        }
        previous.copy_from_slice(line);
    }
    let table = crc_table();
    let mut out = vec![0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
    let mut chunk = |kind: &[u8; 4], body: &[u8]| {
        out.extend_from_slice(&(body.len() as u32).to_be_bytes());
        out.extend_from_slice(kind);
        out.extend_from_slice(body);
        out.extend_from_slice(&crc32(&table, &[kind, body]).to_be_bytes());
    };
    let mut header = Vec::with_capacity(13);
    header.extend_from_slice(&width.to_be_bytes());
    header.extend_from_slice(&height.to_be_bytes());
    header.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(b"IHDR", &header);
    chunk(b"IDAT", &zlib(&raw));
    chunk(b"IEND", &[]);
    out
}

#[cfg(test)]
pub(crate) mod inflate {
    //! A small decoder, only for tests: the bytes must come back exactly.

    struct Bits<'a> {
        data: &'a [u8],
        at: usize,
        bit: u32,
    }

    impl Bits<'_> {
        fn get(&mut self, n: u32) -> u32 {
            let mut v = 0;
            for i in 0..n {
                let b = (self.data[self.at] >> self.bit) & 1;
                v |= (b as u32) << i;
                self.bit += 1;
                if self.bit == 8 {
                    self.bit = 0;
                    self.at += 1;
                }
            }
            v
        }

        fn code(&mut self, n: u32) -> u32 {
            let mut v = 0;
            for _ in 0..n {
                v = (v << 1) | self.get(1);
            }
            v
        }
    }

    fn symbol(b: &mut Bits<'_>) -> u32 {
        let mut c = b.code(7);
        if c <= 0b0010111 {
            return 256 + c;
        }
        c = (c << 1) | b.get(1);
        if (0x30..=0xBF).contains(&c) {
            return c - 0x30;
        }
        if (0xC0..=0xC7).contains(&c) {
            return 280 + c - 0xC0;
        }
        c = (c << 1) | b.get(1);
        144 + c - 0x190
    }

    pub fn inflate_zlib(z: &[u8]) -> Vec<u8> {
        assert_eq!(z[0] & 0x0F, 8, "deflate");
        assert_eq!((z[0] as u32 * 256 + z[1] as u32) % 31, 0, "header check");
        let mut b = Bits { data: &z[2..], at: 0, bit: 0 };
        assert_eq!(b.get(1), 1, "one last block");
        assert_eq!(b.get(2), 1, "fixed codes");
        let mut out: Vec<u8> = Vec::new();
        loop {
            let s = symbol(&mut b);
            if s < 256 {
                out.push(s as u8);
            } else if s == 256 {
                break;
            } else {
                let l = (s - 257) as usize;
                let len = super::LENGTH_BASE[l] as usize + b.get(super::LENGTH_EXTRA[l] as u32) as usize;
                let d = b.code(5) as usize;
                let dist = super::DIST_BASE[d] as usize + b.get(super::DIST_EXTRA[d] as u32) as usize;
                for _ in 0..len {
                    out.push(out[out.len() - dist]);
                }
            }
        }
        out
    }

    /// Decodes a PNG made by this crate, checking every chunk's checksum.
    pub fn decode_png(png: &[u8]) -> (u32, u32, Vec<u8>) {
        assert_eq!(&png[..8], &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A]);
        let mut at = 8;
        let (mut w, mut h) = (0, 0);
        let mut idat = Vec::new();
        let table = super::crc_table();
        while at < png.len() {
            let len = u32::from_be_bytes(png[at..at + 4].try_into().unwrap()) as usize;
            let kind = &png[at + 4..at + 8];
            let body = &png[at + 8..at + 8 + len];
            let crc = u32::from_be_bytes(png[at + 8 + len..at + 12 + len].try_into().unwrap());
            assert_eq!(crc, super::crc32(&table, &[kind, body]), "chunk checksum");
            if kind == b"IHDR" {
                w = u32::from_be_bytes(body[0..4].try_into().unwrap());
                h = u32::from_be_bytes(body[4..8].try_into().unwrap());
                assert_eq!(&body[8..13], &[8, 2, 0, 0, 0]);
            } else if kind == b"IDAT" {
                idat.extend_from_slice(body);
            }
            at += 12 + len;
        }
        let raw = inflate_zlib(&idat);
        let stride = w as usize * 3;
        assert_eq!(raw.len(), (stride + 1) * h as usize);
        let mut out = vec![0u8; stride * h as usize];
        for y in 0..h as usize {
            let filter = raw[y * (stride + 1)];
            for x in 0..stride {
                let v = raw[y * (stride + 1) + 1 + x];
                let left = if x >= 3 { out[y * stride + x - 3] } else { 0 };
                let above = if y > 0 { out[(y - 1) * stride + x] } else { 0 };
                out[y * stride + x] = match filter {
                    1 => v.wrapping_add(left),
                    2 => v.wrapping_add(above),
                    other => panic!("filter {other}"),
                };
            }
        }
        (w, h, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lfsr(seed: u32, n: usize) -> Vec<u8> {
        let mut s = seed | 1;
        (0..n)
            .map(|_| {
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                s as u8
            })
            .collect()
    }

    #[test]
    fn deflate_round_trips() {
        let mut cases: Vec<Vec<u8>> = vec![vec![], vec![7], vec![1, 2], vec![9; 3], vec![0; 100_000], lfsr(3, 5000)];
        cases.push(b"the quick brown fox the quick brown fox the quick brown fox".to_vec());
        // Repeats at distances that cross every code boundary.
        for d in [1, 2, 3, 5, 9, 33, 257, 4097, 16385, 24577, 32768] {
            let mut v = lfsr(d as u32 + 1, d);
            for i in 0..600 {
                let b = v[v.len() - d + (i % d).min(0)];
                v.push(b);
            }
            cases.push(v);
        }
        for data in cases {
            assert_eq!(inflate::inflate_zlib(&zlib(&data)), data, "length {}", data.len());
        }
    }

    #[test]
    fn known_checksums() {
        assert_eq!(crc32(&crc_table(), &[b"IEND"]), 0xAE42_6082);
        assert_eq!(adler32(b"Wikipedia"), 0x11E6_0398);
    }

    #[test]
    fn png_round_trips_and_compresses_flat_colour() {
        let (w, h) = (200u32, 120u32);
        let mut rgb = Vec::new();
        for y in 0..h {
            for x in 0..w {
                let inside = (x as i32 - 100).pow(2) + (y as i32 - 60).pow(2) < 2500;
                rgb.extend_from_slice(if inside { &[200, 90, 40] } else { &[245, 245, 240] });
            }
        }
        let png = encode_rgb(w, h, &rgb);
        let (a, b, back) = inflate::decode_png(&png);
        assert_eq!((a, b), (w, h));
        assert_eq!(back, rgb);
        assert!(png.len() < rgb.len() / 20, "{} bytes from {}", png.len(), rgb.len());
    }
}
